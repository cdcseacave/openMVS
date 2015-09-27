#include "minimize.inl"
#include "ordering.inl"
#include "treeProbabilities.inl"

static void DefaultErrorFn(const char* msg)
{
	fprintf(stderr, "%s\n", msg);
	exit(1);
}

template <class T> MRFEnergy<T>::MRFEnergy(GlobalSize Kglobal, ErrorFunction errorFn)
	: m_errorFn(errorFn ? errorFn : DefaultErrorFn),
	  m_mallocBlockFirst(NULL),
	  m_nodeFirst(NULL),
	  m_nodeLast(NULL),
	  m_nodeNum(0),
	  m_edgeNum(0),
	  m_Kglobal(Kglobal),
	  m_vectorMaxSizeInBytes(0),
	  m_isEnergyConstructionCompleted(false),
	  m_buf(NULL)
{
}

template <class T> MRFEnergy<T>::~MRFEnergy()
{
	while (m_mallocBlockFirst)
	{
		MallocBlock* next = m_mallocBlockFirst->m_next;
		delete m_mallocBlockFirst;
		m_mallocBlockFirst = next;
	}
}

template <class T> typename MRFEnergy<T>::NodeId MRFEnergy<T>::AddNode(LocalSize K, NodeData data)
{
	if (m_isEnergyConstructionCompleted)
	{
		m_errorFn("Error in AddNode(): graph construction completed - nodes cannot be added");
	}

	int actualVectorSize = Vector::GetSizeInBytes(m_Kglobal, K);
	if (actualVectorSize < 0)
	{
		m_errorFn("Error in AddNode() (invalid parameter?)");
	}
	if (m_vectorMaxSizeInBytes < actualVectorSize)
	{
		m_vectorMaxSizeInBytes = actualVectorSize;
	}
	int nodeSize = sizeof(Node) - sizeof(Vector) + actualVectorSize;
	Node* i = (Node *) Malloc(nodeSize);

	i->m_K = K;
	i->m_D.Initialize(m_Kglobal, K, data);

	i->m_firstForward = NULL;
	i->m_firstBackward = NULL;
	i->m_prev = m_nodeLast;
	if (m_nodeLast)
	{
		m_nodeLast->m_next = i;
	}
	else
	{
		m_nodeFirst = i;
	}
	m_nodeLast = i;
	i->m_next = NULL;

	i->m_ordering = m_nodeNum ++;

	return i;
}

template <class T> void MRFEnergy<T>::AddNodeData(NodeId i, NodeData data)
{
	i->m_D.Add(m_Kglobal, i->m_K, data);
}

template <class T> void MRFEnergy<T>::AddEdge(NodeId i, NodeId j, EdgeData data)
{
	if (m_isEnergyConstructionCompleted)
	{
		m_errorFn("Error in AddNode(): graph construction completed - nodes cannot be added");
	}

	MRFEdge* e;

	int actualEdgeSize = Edge::GetSizeInBytes(m_Kglobal, i->m_K, j->m_K, data);
	if (actualEdgeSize < 0)
	{
		m_errorFn("Error in AddEdge() (invalid parameter?)");
	}
	int MRFedgeSize = sizeof(MRFEdge) - sizeof(Edge) + actualEdgeSize;
	e = (MRFEdge*) Malloc(MRFedgeSize);

	e->m_message.Initialize(m_Kglobal, i->m_K, j->m_K, data, &i->m_D, &j->m_D);

	e->m_tail = i;
	e->m_nextForward = i->m_firstForward;
	i->m_firstForward = e;

	e->m_head = j;
	e->m_nextBackward = j->m_firstBackward;
	j->m_firstBackward = e;

	m_edgeNum ++;
}

/////////////////////////////////////////////////////////////////////////////////

template <class T> void MRFEnergy<T>::ZeroMessages()
{
	Node* i;
	MRFEdge* e;

	if (!m_isEnergyConstructionCompleted)
	{
		CompleteGraphConstruction();
	}

	for (i=m_nodeFirst; i; i=i->m_next)
	{
		for (e=i->m_firstForward; e; e=e->m_nextForward)
		{
			e->m_message.GetMessagePtr()->SetZero(m_Kglobal, i->m_K);
		}
	}
}

template <class T> void MRFEnergy<T>::AddRandomMessages(unsigned int random_seed, REAL min_value, REAL max_value)
{
	Node* i;
	MRFEdge* e;
	int k;

	if (!m_isEnergyConstructionCompleted)
	{
		CompleteGraphConstruction();
	}

	srand(random_seed);

	for (i=m_nodeFirst; i; i=i->m_next)
	{
		for (e=i->m_firstForward; e; e=e->m_nextForward)
		{
			Vector* M = e->m_message.GetMessagePtr();
			for (k=0; k<M->GetArraySize(m_Kglobal, i->m_K); k++)
			{
				REAL x = (REAL)( min_value + rand()/((double)RAND_MAX) * (max_value - min_value) );
				x += M->GetArrayValue(m_Kglobal, i->m_K, k);
				M->SetArrayValue(m_Kglobal, i->m_K, k, x);
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////

template <class T> void MRFEnergy<T>::CompleteGraphConstruction()
{
	Node* i;
	Node* j;
	MRFEdge* e;
	MRFEdge* ePrev;

	if (m_isEnergyConstructionCompleted)
	{
		m_errorFn("Fatal error in CompleteGraphConstruction");
	}

	printf("Completing graph construction... ");

	if (m_buf)
	{
		m_errorFn("CompleteGraphConstruction(): fatal error");
	}

	m_buf = (char *) Malloc(m_vectorMaxSizeInBytes + 
		( m_vectorMaxSizeInBytes > Edge::GetBufSizeInBytes(m_vectorMaxSizeInBytes) ?
		  m_vectorMaxSizeInBytes : Edge::GetBufSizeInBytes(m_vectorMaxSizeInBytes) ) );

	// set forward and backward edges properly
#ifdef _DEBUG
	int ordering;
	for (i=m_nodeFirst, ordering=0; i; i=i->m_next, ordering++)
	{
		if ( (i->m_ordering != ordering)
		  || (i->m_ordering == 0 && i->m_prev)
		  || (i->m_ordering != 0 && i->m_prev->m_ordering != ordering-1) )
		{
			m_errorFn("CompleteGraphConstruction(): fatal error (wrong ordering)");
		}
	}
	if (ordering != m_nodeNum)
	{
		m_errorFn("CompleteGraphConstruction(): fatal error");
	}
#endif
	for (i=m_nodeFirst; i; i=i->m_next)
	{
		i->m_firstBackward = NULL;
	}
	for (i=m_nodeFirst; i; i=i->m_next)
	{
		ePrev = NULL;
		for (e=i->m_firstForward; e; )
		{
			assert(i == e->m_tail);
			j = e->m_head;

			if (i->m_ordering < j->m_ordering)
			{
				e->m_nextBackward = j->m_firstBackward;
				j->m_firstBackward = e;

				ePrev = e;
				e = e->m_nextForward;
			}
			else
			{
				e->m_message.Swap(m_Kglobal, i->m_K, j->m_K);
				e->m_tail = j;
				e->m_head = i;

				MRFEdge* eNext = e->m_nextForward;

				if (ePrev)
				{
					ePrev->m_nextForward = e->m_nextForward;
				}
				else
				{
					i->m_firstForward = e->m_nextForward;
				}

				e->m_nextForward = j->m_firstForward;
				j->m_firstForward = e;

				e->m_nextBackward = i->m_firstBackward;
				i->m_firstBackward = e;

				e = eNext;
			}
		}
	}

	m_isEnergyConstructionCompleted = true;

	// ZeroMessages();

	printf("done\n");
}
