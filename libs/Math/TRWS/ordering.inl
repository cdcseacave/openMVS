template <class T> void MRFEnergy<T>::SetAutomaticOrdering()
{
	int dMin;
	Node* i;
	Node* iMin;
	Node* list;
	Node* listBoundary;
	MRFEdge* e;

	if (m_isEnergyConstructionCompleted)
	{
		m_errorFn("Error in SetAutomaticOrdering(): function cannot be called after graph construction is completed");
	}

	printf("Setting automatic ordering... ");

	list = m_nodeFirst;
	listBoundary = NULL;
	m_nodeFirst = m_nodeLast = NULL;
	for (i=list; i; i=i->m_next)
	{
		i->m_ordering = 2*m_nodeNum; // will contain remaining degree mod m_nodeNum (i.e. number of edges connecting to nodes in 'listBoundary' and 'list')
		                             // if i->m_ordering \in [2*m_nodeNum;  3*m_nodeNum) - not assigned yet, belongs to 'list'
		                             // if i->m_ordering \in [m_nodeNum;    2*m_nodeNum) - not assigned yet, belongs to 'listBoundary'
		                             // if i->m_ordering \in [0;            m_nodeNum  ) - assigned, belongs to 'm_nodeFirst'
		for (e=i->m_firstForward; e; e=e->m_nextForward)
		{
			i->m_ordering ++;
		}
		for (e=i->m_firstBackward; e; e=e->m_nextBackward)
		{
			i->m_ordering ++;
		}
	}

	while (list)
	{
		// find node with the smallest remaining degree in list
		dMin = m_nodeNum;
		for (i=list; i; i=i->m_next)
		{
			assert(i->m_ordering >= 2*m_nodeNum);
			if (dMin > i->m_ordering - 2*m_nodeNum)
			{
				dMin = i->m_ordering - 2*m_nodeNum;
				iMin = i;
			}
		}
		i = iMin;

		// remove i from list
		if (i->m_prev) i->m_prev->m_next = i->m_next;
		else           list              = i->m_next;
		if (i->m_next) i->m_next->m_prev = i->m_prev;

		// add i to listBoundary
		listBoundary = i;
		i->m_prev = NULL;
		i->m_next = NULL;
		i->m_ordering -= m_nodeNum;

		while (listBoundary)
		{
			// find node with the smallest remaining degree in listBoundary
			dMin = m_nodeNum;
			for (i=listBoundary; i; i=i->m_next)
			{
				assert(i->m_ordering >= m_nodeNum && i->m_ordering < 2*m_nodeNum);
				if (dMin > i->m_ordering - m_nodeNum)
				{
					dMin = i->m_ordering - m_nodeNum;
					iMin = i;
				}
			}
			i = iMin;

			// remove i from listBoundary
			if (i->m_prev) i->m_prev->m_next = i->m_next;
			else           listBoundary      = i->m_next;
			if (i->m_next) i->m_next->m_prev = i->m_prev;

			// add i to m_nodeFirst
			if (m_nodeLast)
			{
				m_nodeLast->m_next = i;
				i->m_ordering = m_nodeLast->m_ordering + 1;
			}
			else
			{
				m_nodeFirst = i;
				i->m_ordering = 0;
			}
			i->m_prev = m_nodeLast;
			m_nodeLast = i;
			i->m_next = NULL;

			// process neighbors of i=m_nodeLast: decrease their remaining degree,
			// put them into listBoundary (if they are in list)
			for (e=m_nodeLast->m_firstForward; e; e=e->m_nextForward)
			{
				assert(m_nodeLast == e->m_tail);
				i = e->m_head;
				if (i->m_ordering >= m_nodeNum)
				{
					i->m_ordering --; // decrease remaining degree of i
					if (i->m_ordering >= 2*m_nodeNum)
					{
						// remove i from list
						if (i->m_prev) i->m_prev->m_next = i->m_next;
						else           list              = i->m_next;
						if (i->m_next) i->m_next->m_prev = i->m_prev;

						// add i to listBoundary
						if (listBoundary) listBoundary->m_prev = i;
						i->m_prev = NULL;
						i->m_next = listBoundary;
						listBoundary = i;
						i->m_ordering -= m_nodeNum;
					}
				}
			}
			for (e=m_nodeLast->m_firstBackward; e; e=e->m_nextBackward)
			{
				assert(m_nodeLast == e->m_head);
				i = e->m_tail;
				if (i->m_ordering >= m_nodeNum)
				{
					i->m_ordering --; // decrease remaining degree of i
					if (i->m_ordering >= 2*m_nodeNum)
					{
						// remove i from list
						if (i->m_prev) i->m_prev->m_next = i->m_next;
						else           list              = i->m_next;
						if (i->m_next) i->m_next->m_prev = i->m_prev;

						// add i to listBoundary
						if (listBoundary) listBoundary->m_prev = i;
						i->m_prev = NULL;
						i->m_next = listBoundary;
						listBoundary = i;
						i->m_ordering -= m_nodeNum;
					}
				}
			}
		}
	}

	printf("done\n");

	CompleteGraphConstruction();
}
