/******************************************************************
typeTruncatedQuadratic2D.h

Energy function with 2-dimensional truncated quadratic interactions:
   E(x)   =   \sum_i D_i(x_i)   +   \sum_ij V_ij(x_i,x_j)
   where x_i \in {0, 1, ..., KX-1} x {0, 1, ..., KY-1}
   (i.e. x_i = (x_i(1), x_i(2))).
   V_ij(ki, kj) = min { alpha1_ij*(ki(1)-kj(2))^2 + alpha2_ij*(ki(2)-kj(2))^2, lambda_ij }.
   alpha1_ij, alpha2_ij and lambda_ij must be non-negative.

Example usage:

Minimize function E(x,y) = Dx(x) + Dy(y) + min { alpha1*(x(1) - y(1))^2 + alpha2*(x(2) - y(2))^2, lambda } 
where 
  x,y \in {0,1} x {0,1,2}
  Dx(0,0) = 0, Dx(0,1) = 1, Dx(0,2) = 2,
  Dx(1,0) = 3, Dx(1,1) = 4, Dx(1,2) = 5,
  Dy(y) = 0 for all y,
  alpha1 = 6,
  alpha2 = 7,
  lambda = 8




#include <stdio.h>
#include "MRFEnergy.h"

void testTruncatedQuadratic2D()
{
	MRFEnergy<TypeTruncatedQuadratic2D>* mrf;
	MRFEnergy<TypeTruncatedQuadratic2D>::NodeId* nodes;
	MRFEnergy<TypeTruncatedQuadratic2D>::Options options;
	TypeTruncatedQuadratic2D::REAL energy, lowerBound;

	const int nodeNum = 2; // number of nodes
	const int KX = 2; //   label
	const int KY = 3; // dimensions
	const int K = KX*KY; // number of labels
	TypeTruncatedQuadratic2D::REAL D[K];
	TypeTruncatedQuadratic2D::Label x, y;

	mrf = new MRFEnergy<TypeTruncatedQuadratic2D>(TypeTruncatedQuadratic2D::GlobalSize(KX, KY));
	nodes = new MRFEnergy<TypeTruncatedQuadratic2D>::NodeId[nodeNum];

	// construct energy
	D[0] = 0; D[1] = 1; D[2] = 2;
	D[3] = 3; D[4] = 4; D[5] = 5;
	nodes[0] = mrf->AddNode(TypeTruncatedQuadratic2D::LocalSize(), TypeTruncatedQuadratic2D::NodeData(D));
	D[0] = 0; D[1] = 0; D[2] = 0;
	D[3] = 0; D[4] = 0; D[5] = 0;
	nodes[1] = mrf->AddNode(TypeTruncatedQuadratic2D::LocalSize(), TypeTruncatedQuadratic2D::NodeData(D));
	mrf->AddEdge(nodes[0], nodes[1], TypeTruncatedQuadratic2D::EdgeData(6, 7, 8));

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

	printf("Solution: (%d, %d) (%d, %d)\n", x.m_kx, x.m_ky, y.m_kx, y.m_ky);

	// done
	delete nodes;
	delete mrf;
}

*******************************************************************/


















#ifndef __TYPETRUNCATEDQUADRATIC2D_H__
#define __TYPETRUNCATEDQUADRATIC2D_H__

#include <string.h>
#include <assert.h>


template <class T> class MRFEnergy;


class TypeTruncatedQuadratic2D
{
private:
	struct Vector; // node parameters and messages
	struct Edge; // stores edge information and either forward or backward message

public:
	// types declarations
	struct Label
	{
		int		m_kx, m_ky;
	};
	typedef double REAL;
	struct GlobalSize; // global information about number of labels
	struct LocalSize; // local information about number of labels (stored at each node)
	struct NodeData; // argument to MRFEnergy::AddNode()
	struct EdgeData; // argument to MRFEnergy::AddEdge()


	struct GlobalSize
	{
		GlobalSize(int KX, int KY);

	private:
	friend struct Vector;
	friend struct Edge;
		int		m_KX, m_KY; // label dimensions
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
		EdgeData(REAL alphaX, REAL alphaY, REAL lambda);

	private:
	friend struct Vector;
	friend struct Edge;
		REAL		m_alphaX, m_alphaY;
		REAL		m_lambda;
	};







	//////////////////////////////////////////////////////////////////////////////////
	////////////////////////// Visible only to MRFEnergy /////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////

private:
friend class MRFEnergy<TypeTruncatedQuadratic2D>;

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
		void DistanceTransformL2(int K, int stride, REAL alpha, REAL* source, REAL* dest, int* parabolas, int* intersections);

		// edge information
		REAL		m_alphaX, m_alphaY;
		REAL		m_lambda;

		// message
		Vector		m_message;
	};
};




//////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Implementation ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


inline TypeTruncatedQuadratic2D::GlobalSize::GlobalSize(int KX, int KY)
{
	m_KX = KX;
	m_KY = KY;
	m_K = KX*KY;
}

///////////////////// NodeData and EdgeData ///////////////////////

inline TypeTruncatedQuadratic2D::NodeData::NodeData(REAL* data)
{
	m_data = data;
}

inline TypeTruncatedQuadratic2D::EdgeData::EdgeData(REAL alphaX, REAL alphaY, REAL lambda)
{
	m_alphaX = alphaX;
	m_alphaY = alphaY;
	m_lambda = lambda;
}

///////////////////// Vector ///////////////////////

inline int TypeTruncatedQuadratic2D::Vector::GetSizeInBytes(GlobalSize Kglobal, LocalSize K)
{
	if (Kglobal.m_KX < 1 || Kglobal.m_KY < 1)
	{
		return -1;
	}
	return Kglobal.m_K*sizeof(REAL);
}
inline void TypeTruncatedQuadratic2D::Vector::Initialize(GlobalSize Kglobal, LocalSize K, NodeData data)
{
	memcpy(m_data, data.m_data, Kglobal.m_K*sizeof(REAL));
}

inline void TypeTruncatedQuadratic2D::Vector::Add(GlobalSize Kglobal, LocalSize K, NodeData data)
{
	for (int k=0; k<Kglobal.m_K; k++)
	{
		m_data[k] += data.m_data[k];
	}
}

inline void TypeTruncatedQuadratic2D::Vector::SetZero(GlobalSize Kglobal, LocalSize K)
{
	memset(m_data, 0, Kglobal.m_K*sizeof(REAL));
}

inline void TypeTruncatedQuadratic2D::Vector::Copy(GlobalSize Kglobal, LocalSize K, Vector* V)
{
	memcpy(m_data, V->m_data, Kglobal.m_K*sizeof(REAL));
}

inline void TypeTruncatedQuadratic2D::Vector::Add(GlobalSize Kglobal, LocalSize K, Vector* V)
{
	for (int k=0; k<Kglobal.m_K; k++)
	{
		m_data[k] += V->m_data[k];
	}
}

inline TypeTruncatedQuadratic2D::REAL TypeTruncatedQuadratic2D::Vector::GetValue(GlobalSize Kglobal, LocalSize K, Label k)
{
	assert(k.m_kx>=0 && k.m_kx<Kglobal.m_KX && k.m_ky>=0 && k.m_ky<Kglobal.m_KY);
	return m_data[k.m_kx + k.m_ky*Kglobal.m_KX];
}

inline TypeTruncatedQuadratic2D::REAL TypeTruncatedQuadratic2D::Vector::ComputeMin(GlobalSize Kglobal, LocalSize K, Label& _kMin)
{
	REAL vMin = m_data[0];
	int kMin = 0;
	for (int k=1; k<Kglobal.m_K; k++)
	{
		if (vMin > m_data[k])
		{
			vMin = m_data[k];
			kMin = k;
		}
	}

	_kMin.m_ky = kMin / Kglobal.m_KX;
	_kMin.m_kx = kMin - _kMin.m_ky * Kglobal.m_KX;

	return vMin;
}

inline TypeTruncatedQuadratic2D::REAL TypeTruncatedQuadratic2D::Vector::ComputeAndSubtractMin(GlobalSize Kglobal, LocalSize K)
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

inline int TypeTruncatedQuadratic2D::Vector::GetArraySize(GlobalSize Kglobal, LocalSize K)
{
	return Kglobal.m_K;
}

inline TypeTruncatedQuadratic2D::REAL TypeTruncatedQuadratic2D::Vector::GetArrayValue(GlobalSize Kglobal, LocalSize K, int k)
{
	assert(k>=0 && k<Kglobal.m_K);
	return m_data[k];
}

inline void TypeTruncatedQuadratic2D::Vector::SetArrayValue(GlobalSize Kglobal, LocalSize K, int k, REAL x)
{
	assert(k>=0 && k<Kglobal.m_K);
	m_data[k] = x;
}

///////////////////// EdgeDataAndMessage implementation /////////////////////////

inline int TypeTruncatedQuadratic2D::Edge::GetSizeInBytes(GlobalSize Kglobal, LocalSize Ki, LocalSize Kj, EdgeData data)
{
	if (data.m_alphaX < 0 || data.m_alphaY < 0 || data.m_lambda < 0)
	{
		return -1;
	}
	return sizeof(Edge) - sizeof(Vector) + Kglobal.m_K*sizeof(REAL);
}

inline int TypeTruncatedQuadratic2D::Edge::GetBufSizeInBytes(int vectorMaxSizeInBytes)
{
	int K = vectorMaxSizeInBytes / sizeof(REAL);
	return K*sizeof(REAL) + (2*K+1)*sizeof(int);
}

inline void TypeTruncatedQuadratic2D::Edge::Initialize(GlobalSize Kglobal, LocalSize Ki, LocalSize Kj, EdgeData data, Vector* Di, Vector* Dj)
{
	m_alphaX = data.m_alphaX;
	m_alphaY = data.m_alphaY;
	m_lambda = data.m_lambda;
	memset(m_message.m_data, 0, Kglobal.m_K*sizeof(REAL));
}

inline TypeTruncatedQuadratic2D::Vector* TypeTruncatedQuadratic2D::Edge::GetMessagePtr()
{
	return &m_message;
}

inline void TypeTruncatedQuadratic2D::Edge::Swap(GlobalSize Kglobal, LocalSize Ki, LocalSize Kj)
{
}

inline TypeTruncatedQuadratic2D::REAL TypeTruncatedQuadratic2D::Edge::UpdateMessage(GlobalSize Kglobal, LocalSize Ksource, LocalSize Kdest, Vector* source, REAL gamma, int dir, void* _buf)
{
	REAL* buf = (REAL*) _buf;
	int* parabolas = (int*) ((char*)_buf + Kglobal.m_K*sizeof(REAL));
	int* intersections = (int*) ((char*)_buf + Kglobal.m_K*(sizeof(REAL) + sizeof(int)));
	Label k;
	REAL vMin;
	REAL* sourcePtr;
	REAL* destPtr;

	m_message.m_data[0] = gamma*source->m_data[0] - m_message.m_data[0];
	vMin = m_message.m_data[0];

	sourcePtr = source->m_data;
	destPtr = m_message.m_data;
	for (sourcePtr++, destPtr++ ; sourcePtr<&source->m_data[Kglobal.m_K]; sourcePtr++, destPtr++)
	{
		destPtr[0] = gamma*sourcePtr[0] - destPtr[0];
		if (vMin > destPtr[0])
		{
			vMin = destPtr[0];
		}
	}

	// distance transform in 'X' direction
	for (k.m_ky=0; k.m_ky<Kglobal.m_KY; k.m_ky++)
	{
		DistanceTransformL2(Kglobal.m_KX, 1, m_alphaX, &m_message.m_data[k.m_ky*Kglobal.m_KX], buf+k.m_ky*Kglobal.m_KX, parabolas, intersections);
	}

	// distance transform in 'Y' direction
	for (k.m_kx=0; k.m_kx<Kglobal.m_KX; k.m_kx++)
	{
		DistanceTransformL2(Kglobal.m_KY, Kglobal.m_KX, m_alphaY, buf+k.m_kx, &m_message.m_data[k.m_kx], parabolas, intersections);
	}

	for (destPtr=m_message.m_data; destPtr<&m_message.m_data[Kglobal.m_K]; destPtr++)
	{
		destPtr[0] -= vMin;
		if (destPtr[0] > m_lambda)
		{
			destPtr[0] = m_lambda;
		}
	}

	return vMin;
}

inline void TypeTruncatedQuadratic2D::Edge::AddColumn(GlobalSize Kglobal, LocalSize Ksource, LocalSize Kdest, Label ksource, Vector* dest, int dir)
{
	assert(ksource.m_kx>=0 && ksource.m_kx<Kglobal.m_KX && ksource.m_ky>=0 && ksource.m_ky<Kglobal.m_KY);

	REAL* destPtr;
	Label k;

	destPtr = dest->m_data;
	for (k.m_ky=0; k.m_ky<Kglobal.m_KY; k.m_ky++)
	for (k.m_kx=0; k.m_kx<Kglobal.m_KX; k.m_kx++, destPtr++)
	{
		REAL cost = m_alphaX*(k.m_kx - ksource.m_kx)*(k.m_kx - ksource.m_kx)
		          + m_alphaY*(k.m_ky - ksource.m_ky)*(k.m_ky - ksource.m_ky);
		destPtr[0] += (cost < m_lambda) ? cost : m_lambda;
	}
}

//////////////////////////////////////////////////////////////////////////////////



inline void TypeTruncatedQuadratic2D::Edge::DistanceTransformL2(int K, int stride, REAL alpha, REAL* source, REAL* dest, int* parabolas, int* intersections)
{
	assert(alpha >= 0);

	if (alpha == 0)
	{
		REAL* ptr;
		REAL vMin = source[0];

		for (ptr=source+stride; ptr<source+K*stride; ptr+=stride)
		{
			if (vMin > ptr[0])
			{
				vMin = ptr[0];
			}
		}
		for (ptr=dest; ptr<dest+K*stride; ptr+=stride)
		{
			ptr[0] = vMin;
		}
		return;
	}

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
			k = (int) ( 1 + ( (i + j)  +  (source[stride*j] - source[stride*i]) / (alpha * (j - i)) ) / 2);
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
			dest[stride*i] = source[stride*j] + alpha*(i-j)*(i-j);
		}
	}
}

#endif
