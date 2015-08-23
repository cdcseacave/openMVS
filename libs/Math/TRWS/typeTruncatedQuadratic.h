/******************************************************************
typeTruncatedQuadratic.h

Energy function with truncated quadratic interactions:
   E(x)   =   \sum_i D_i(x_i)   +   \sum_ij V_ij(x_i,x_j)
   where x_i \in {0, 1, ..., K-1},
   V_ij(ki, kj) = min { alpha_ij*(ki-kj)^2, lambda_ij }.
   alpha_ij and lambda_ij must be non-negative.

Example usage:

Minimize function E(x,y) = Dx(x) + Dy(y) + min { alpha*(x - y)^2 , lambda } where 
  x,y \in {0,1,2},
  Dx(0) = 0, Dx(1) = 1, Dx(2) = 2,
  Dy(0) = 3, Dy(1) = 4, Dy(2) = 5,
  alpha = 6,
  lambda = 7




#include <stdio.h>
#include "MRFEnergy.h"

void testTruncatedQuadratic()
{
	MRFEnergy<TypeTruncatedQuadratic>* mrf;
	MRFEnergy<TypeTruncatedQuadratic>::NodeId* nodes;
	MRFEnergy<TypeTruncatedQuadratic>::Options options;
	TypeTruncatedQuadratic::REAL energy, lowerBound;

	const int nodeNum = 2; // number of nodes
	const int K = 3; // number of labels
	TypeTruncatedQuadratic::REAL D[K];
	int x, y;

	mrf = new MRFEnergy<TypeTruncatedQuadratic>(TypeTruncatedQuadratic::GlobalSize(K));
	nodes = new MRFEnergy<TypeTruncatedQuadratic>::NodeId[nodeNum];

	// construct energy
	D[0] = 0; D[1] = 1; D[2] = 2;
	nodes[0] = mrf->AddNode(TypeTruncatedQuadratic::LocalSize(), TypeTruncatedQuadratic::NodeData(D));
	D[0] = 3; D[1] = 4; D[2] = 5;
	nodes[1] = mrf->AddNode(TypeTruncatedQuadratic::LocalSize(), TypeTruncatedQuadratic::NodeData(D));
	mrf->AddEdge(nodes[0], nodes[1], TypeTruncatedQuadratic::EdgeData(6, 7));

	// Function below is optional - it may help if, for example, nodes are added in a random order
	// mrf->SetAutomaticOrdering();

	/////////////////////// TRW-S algorithm //////////////////////
	options.m_iterMax = 30; // maximum number of iterations
	mrf->Minimize_TRW_S(options, lowerBound, energy);

	// read solution
	x = mrf->GetSolution(nodes[0]);
	y = mrf->GetSolution(nodes[1]);

	printf("Solution: %d %d\n", x, y);

	//////////////////////// BP algorithm ////////////////////////
	mrf->ZeroMessages(); // in general not necessary - it may be faster to start 
	                     // with messages computed in previous iterations

	options.m_iterMax = 30; // maximum number of iterations
	mrf->Minimize_BP(options, energy);

	// read solution
	x = mrf->GetSolution(nodes[0]);
	y = mrf->GetSolution(nodes[1]);

	printf("Solution: %d %d\n", x, y);

	// done
	delete nodes;
	delete mrf;
}

*******************************************************************/


















#ifndef __TYPETRUNCATEDQUADRATIC_H__
#define __TYPETRUNCATEDQUADRATIC_H__

#include <string.h>
#include <assert.h>


template <class T> class MRFEnergy;


class TypeTruncatedQuadratic
{
private:
	struct Vector; // node parameters and messages
	struct Edge; // stores edge information and either forward or backward message

public:
	// types declarations
	typedef int Label;
	typedef double REAL;
	struct GlobalSize; // global information about number of labels
	struct LocalSize; // local information about number of labels (stored at each node)
	struct NodeData; // argument to MRFEnergy::AddNode()
	struct EdgeData; // argument to MRFEnergy::AddEdge()


	struct GlobalSize
	{
		GlobalSize(int K);

	private:
	friend struct Vector;
	friend struct Edge;
		int		m_K; // number of labels
	};

	struct LocalSize // number of labels is stored at MRFEnergy::m_Kglobal
	{
	};

	struct NodeData
	{
		NodeData(REAL* data); // data = pointer to array of size MRFEnergy::m_Kglobal

	private:
	friend struct Vector;
	friend struct Edge;
		REAL*		m_data;
	};

	struct EdgeData
	{
		EdgeData(REAL alpha, REAL lambda);

	private:
	friend struct Vector;
	friend struct Edge;
		REAL		m_alpha;
		REAL		m_lambda;
	};







	//////////////////////////////////////////////////////////////////////////////////
	////////////////////////// Visible only to MRFEnergy /////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////

private:
friend class MRFEnergy<TypeTruncatedQuadratic>;

	struct Vector
	{
		static int GetSizeInBytes(GlobalSize Kglobal, LocalSize K); // returns -1 if invalid K's
		void Initialize(GlobalSize Kglobal, LocalSize K, NodeData data);  // called once when user adds a node
		void Add(GlobalSize Kglobal, LocalSize K, NodeData data); // called once when user calls MRFEnergy::AddNodeData()

		void SetZero(GlobalSize Kglobal, LocalSize K);                            // set this[k] = 0
		void Copy(GlobalSize Kglobal, LocalSize K, Vector* V);                    // set this[k] = V[k]
		void Add(GlobalSize Kglobal, LocalSize K, Vector* V);                     // set this[k] = this[k] + V[k]
		REAL GetValue(GlobalSize Kglobal, LocalSize K, Label k);                  // return this[k]
		REAL ComputeMin(GlobalSize Kglobal, LocalSize K, Label& kMin);            // return vMin = min_k { this[k] }, set kMin
		REAL ComputeAndSubtractMin(GlobalSize Kglobal, LocalSize K);              // same as previous, but additionally set this[k] -= vMin (and kMin is not returned)

		static int GetArraySize(GlobalSize Kglobal, LocalSize K);
		REAL GetArrayValue(GlobalSize Kglobal, LocalSize K, int k); // note: k is an integer in [0..GetArraySize()-1].
		                                                            // For Potts functions GetArrayValue() and GetValue() are the same,
		                                                            // but they are different for, say, 2-dimensional labels.
		void SetArrayValue(GlobalSize Kglobal, LocalSize K, int k, REAL x);

	private:
	friend struct Edge;
		REAL		m_data[1]; // actual size is MRFEnergy::m_Kglobal
	};

	struct Edge
	{
		static int GetSizeInBytes(GlobalSize Kglobal, LocalSize Ki, LocalSize Kj, EdgeData data); // returns -1 if invalid data
		static int GetBufSizeInBytes(int vectorMaxSizeInBytes); // returns size of buffer need for UpdateMessage()
		void Initialize(GlobalSize Kglobal, LocalSize Ki, LocalSize Kj, EdgeData data, Vector* Di, Vector* Dj); // called once when user adds an edge
		Vector* GetMessagePtr();
		void Swap(GlobalSize Kglobal, LocalSize Ki, LocalSize Kj); // if the client calls this function, then the meaning of 'dir'
								                                               // in distance transform functions is swapped

		// When UpdateMessage() is called, edge contains message from dest to source.
		// The function must replace it with the message from source to dest.
		// The update rule is given below assuming that source corresponds to tail (i) and dest corresponds
		// to head (j) (which is the case if dir==0).
		//
		// 1. Compute Di[ki] = gamma*source[ki] - message[ki].  (Note: message = message from j to i).
		// 2. Compute distance transform: set
		//       message[kj] = min_{ki} (Di[ki] + V(ki,kj)). (Note: message = message from i to j).
		// 3. Compute vMin = min_{kj} m_message[kj].
		// 4. Set m_message[kj] -= vMin.
		// 5. Return vMin.
		//
		// If dir==1 then source corresponds to j, sink corresponds to i. Then the update rule is
		//
		// 1. Compute Dj[kj] = gamma*source[kj] - message[kj].  (Note: message = message from i to j).
		// 2. Compute distance transform: set
		//       message[ki] = min_{kj} (Dj[kj] + V(ki,kj)). (Note: message = message from j to i).
		// 3. Compute vMin = min_{ki} m_message[ki].
		// 4. Set m_message[ki] -= vMin.
		// 5. Return vMin.
		//
		// If Edge::Swap has been called odd number of times, then the meaning of dir is swapped.
		//
		// Vector 'source' must not be modified. Function may use 'buf' as a temporary storage.
		REAL UpdateMessage(GlobalSize Kglobal, LocalSize Ksource, LocalSize Kdest, Vector* source, REAL gamma, int dir, void* buf);

		// If dir==0, then sets dest[kj] += V(ksource,kj).
		// If dir==1, then sets dest[ki] += V(ki,ksource).
		// If Swap() has been called odd number of times, then the meaning of dir is swapped.
		void AddColumn(GlobalSize Kglobal, LocalSize Ksource, LocalSize Kdest, Label ksource, Vector* dest, int dir);

	private:
		// parabolas must be array of size K
		// intersections must be array of size K+1
		void DistanceTransformL2(int K, REAL* source, REAL* dest, int* parabolas, int* intersections);

		// edge information
		REAL		m_alpha;
		REAL		m_lambda;

		// message
		Vector		m_message;
	};
};




//////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Implementation ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


inline TypeTruncatedQuadratic::GlobalSize::GlobalSize(int K)
{
	m_K = K;
}

///////////////////// NodeData and EdgeData ///////////////////////

inline TypeTruncatedQuadratic::NodeData::NodeData(REAL* data)
{
	m_data = data;
}

inline TypeTruncatedQuadratic::EdgeData::EdgeData(REAL alpha, REAL lambda)
{
	m_alpha = alpha;
	m_lambda = lambda;
}

///////////////////// Vector ///////////////////////

inline int TypeTruncatedQuadratic::Vector::GetSizeInBytes(GlobalSize Kglobal, LocalSize K)
{
	if (Kglobal.m_K < 1)
	{
		return -1;
	}
	return Kglobal.m_K*sizeof(REAL);
}
inline void TypeTruncatedQuadratic::Vector::Initialize(GlobalSize Kglobal, LocalSize K, NodeData data)
{
	memcpy(m_data, data.m_data, Kglobal.m_K*sizeof(REAL));
}

inline void TypeTruncatedQuadratic::Vector::Add(GlobalSize Kglobal, LocalSize K, NodeData data)
{
	for (int k=0; k<Kglobal.m_K; k++)
	{
		m_data[k] += data.m_data[k];
	}
}

inline void TypeTruncatedQuadratic::Vector::SetZero(GlobalSize Kglobal, LocalSize K)
{
	memset(m_data, 0, Kglobal.m_K*sizeof(REAL));
}

inline void TypeTruncatedQuadratic::Vector::Copy(GlobalSize Kglobal, LocalSize K, Vector* V)
{
	memcpy(m_data, V->m_data, Kglobal.m_K*sizeof(REAL));
}

inline void TypeTruncatedQuadratic::Vector::Add(GlobalSize Kglobal, LocalSize K, Vector* V)
{
	for (int k=0; k<Kglobal.m_K; k++)
	{
		m_data[k] += V->m_data[k];
	}
}

inline TypeTruncatedQuadratic::REAL TypeTruncatedQuadratic::Vector::GetValue(GlobalSize Kglobal, LocalSize K, Label k)
{
	assert(k>=0 && k<Kglobal.m_K);
	return m_data[k];
}

inline TypeTruncatedQuadratic::REAL TypeTruncatedQuadratic::Vector::ComputeMin(GlobalSize Kglobal, LocalSize K, Label& kMin)
{
	REAL vMin = m_data[0];
	kMin = 0;
	for (int k=1; k<Kglobal.m_K; k++)
	{
		if (vMin > m_data[k])
		{
			vMin = m_data[k];
			kMin = k;
		}
	}

	return vMin;
}

inline TypeTruncatedQuadratic::REAL TypeTruncatedQuadratic::Vector::ComputeAndSubtractMin(GlobalSize Kglobal, LocalSize K)
{
	REAL vMin = m_data[0];
	for (int k=1; k<Kglobal.m_K; k++)
	{
		if (vMin > m_data[k])
		{
			vMin = m_data[k];
		}
	}
	for (int k=0; k<Kglobal.m_K; k++)
	{
		m_data[k] -= vMin;
	}

	return vMin;
}

inline int TypeTruncatedQuadratic::Vector::GetArraySize(GlobalSize Kglobal, LocalSize K)
{
	return Kglobal.m_K;
}

inline TypeTruncatedQuadratic::REAL TypeTruncatedQuadratic::Vector::GetArrayValue(GlobalSize Kglobal, LocalSize K, int k)
{
	assert(k>=0 && k<Kglobal.m_K);
	return m_data[k];
}

inline void TypeTruncatedQuadratic::Vector::SetArrayValue(GlobalSize Kglobal, LocalSize K, int k, REAL x)
{
	assert(k>=0 && k<Kglobal.m_K);
	m_data[k] = x;
}

///////////////////// EdgeDataAndMessage implementation /////////////////////////

inline int TypeTruncatedQuadratic::Edge::GetSizeInBytes(GlobalSize Kglobal, LocalSize Ki, LocalSize Kj, EdgeData data)
{
	if (data.m_alpha < 0 || data.m_lambda < 0)
	{
		return -1;
	}
	return sizeof(Edge) - sizeof(Vector) + Kglobal.m_K*sizeof(REAL);
}

inline int TypeTruncatedQuadratic::Edge::GetBufSizeInBytes(int vectorMaxSizeInBytes)
{
	int K = vectorMaxSizeInBytes / sizeof(REAL);
	return K*sizeof(REAL) + (2*K+1)*sizeof(int);
}

inline void TypeTruncatedQuadratic::Edge::Initialize(GlobalSize Kglobal, LocalSize Ki, LocalSize Kj, EdgeData data, Vector* Di, Vector* Dj)
{
	m_alpha = data.m_alpha;
	m_lambda = data.m_lambda;
	memset(m_message.m_data, 0, Kglobal.m_K*sizeof(REAL));
}

inline TypeTruncatedQuadratic::Vector* TypeTruncatedQuadratic::Edge::GetMessagePtr()
{
	return &m_message;
}

inline void TypeTruncatedQuadratic::Edge::Swap(GlobalSize Kglobal, LocalSize Ki, LocalSize Kj)
{
}

inline TypeTruncatedQuadratic::REAL TypeTruncatedQuadratic::Edge::UpdateMessage(GlobalSize Kglobal, LocalSize Ksource, LocalSize Kdest, Vector* source, REAL gamma, int dir, void* _buf)
{
	REAL* buf = (REAL*) _buf;
	int* parabolas = (int*) ((char*)_buf + Kglobal.m_K*sizeof(REAL));
	int* intersections = (int*) ((char*)_buf + Kglobal.m_K*(sizeof(REAL) + sizeof(int)));
	int k;
	REAL vMin;

	buf[0] = gamma*source->m_data[0] - m_message.m_data[0];
	vMin = buf[0];

	for (k=1; k<Kglobal.m_K; k++)
	{
		buf[k] = gamma*source->m_data[k] - m_message.m_data[k];
		if (vMin > buf[k])
		{
			vMin = buf[k];
		}
	}

	if (m_alpha == 0)
	{
		for (k=0; k<Kglobal.m_K; k++)
		{
			m_message.m_data[k] = 0;
		}
	}
	else
	{
		DistanceTransformL2(Kglobal.m_K, buf, m_message.m_data, parabolas, intersections);

		for (k=0; k<Kglobal.m_K; k++)
		{
			m_message.m_data[k] -= vMin;
			if (m_message.m_data[k] > m_lambda)
			{
				m_message.m_data[k] = m_lambda;
			}
		}
	}

	return vMin;
}

inline void TypeTruncatedQuadratic::Edge::AddColumn(GlobalSize Kglobal, LocalSize Ksource, LocalSize Kdest, Label ksource, Vector* dest, int dir)
{
	assert(ksource>=0 && ksource<Kglobal.m_K);

	int k;

	for (k=0; k<Kglobal.m_K; k++)
	{
		dest->m_data[k] += m_alpha*(k-ksource)*(k-ksource) < m_lambda ? m_alpha*(k-ksource)*(k-ksource) : m_lambda;
	}
}

//////////////////////////////////////////////////////////////////////////////////









inline void TypeTruncatedQuadratic::Edge::DistanceTransformL2(int K, REAL* source, REAL* dest, int* parabolas, int* intersections)
{
	assert(m_alpha > 0);

	int i, j, k, p;
	int r = 0; // = number of parabolas minus 1
	// parabolas[p] will be base of parabola p (0<=p<=r)
	// intersections[p] will be intersection between parabolas p-1 and p (1<=p<=r)
	// intersections[0] will be always 0
	parabolas[0] = 0;
	intersections[0] = 0;

	for (i=1; i<K; i++)
	{
		while ( 1 )
		{
			j = parabolas[r]; // base of previous rightmost visible parabola
			// k is intersection of parabolas i and j
			k = (int) ( 1 + ( (i + j)  +  (source[j] - source[i]) / (m_alpha * (j - i)) ) / 2);
			if (k >= K)
			{
				// i is not visible
				break;
			}
			if (k < 0)
			{
				k = 0;
			}

			if (k > intersections[r])
			{
				// intersection is rightmost, add it to end
				r ++;
				parabolas[r] = i;
				intersections[r] = k;
				break;
			}
			// j is not visible
			if (r == 0)
			{
				parabolas[0] = i;
				break;
			}
			r --;
		}
	}

	intersections[r + 1] = K;

	i = 0;
	for (p=0; p<=r; p++)
	{
		j = parabolas[p];
		// i values in [intersections[p], intersections[p+1]) are assigned to j
		for ( ; i<intersections[p+1]; i++)
		{
			dest[i] = source[j] + m_alpha*(i-j)*(i-j);
		}
	}
}

#endif
