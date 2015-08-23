/******************************************************************
typeBinary.h

Energy function with binary labels:
   E(x)   =   \sum_i D_i(x_i)   +   \sum_ij V_ij(x_i,x_j)
   where x_i \in {0,1}.

Example usage:

Minimize function E(x,y) = Dx(x) + Dy(y) + V(x,y) where 
  x,y \in {0,1},
  Dx(0) = 0, Dx(1) = 1,
  Dy(0) = 2, Dy(1) = 3,
  V(0,0) = 4, V(0,1) = 5,
  V(1,0) = 6, V(1,1) = 7,
  [.] is 1 if it's argument is true, and 0 otherwise.



#include <stdio.h>
#include "MRFEnergy.h"

void testBinary()
{
	MRFEnergy<TypeBinary>* mrf;
	MRFEnergy<TypeBinary>::NodeId* nodes;
	MRFEnergy<TypeBinary>::Options options;
	TypeBinary::REAL energy, lowerBound;

	const int nodeNum = 2; // number of nodes
	int x, y;

	mrf = new MRFEnergy<TypeBinary>(TypeBinary::GlobalSize(K));
	nodes = new MRFEnergy<TypeBinary>::NodeId[nodeNum];

	// construct energy
	nodes[0] = mrf->AddNode(TypeBinary::LocalSize(), TypeBinary::NodeData(0, 1));
	nodes[1] = mrf->AddNode(TypeBinary::LocalSize(), TypeBinary::NodeData(2, 3));
	mrf->AddEdge(nodes[0], nodes[1], TypeBinary::EdgeData(4, 5, 6, 7));

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






#ifndef __TYPEBINARY_H__
#define __TYPEBINARY_H__

#include <string.h>
#include <assert.h>


template <class T> class MRFEnergy;


class TypeBinary
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


	struct GlobalSize // always 2 labels - no need to store it
	{
	};

	struct LocalSize // always 2 labels - no need to store it
	{
	};

	struct NodeData
	{
		NodeData(REAL D0, REAL D1);

	private:
	friend struct Vector;
	friend struct Edge;
		REAL		m_data[2];
	};

	struct EdgeData
	{
		EdgeData(REAL V00, REAL V01, REAL V10, REAL V11);

	private:
	friend struct Vector;
	friend struct Edge;
		REAL		m_V[2][2];
	};







	//////////////////////////////////////////////////////////////////////////////////
	////////////////////////// Visible only to MRFEnergy /////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////

private:
friend class MRFEnergy<TypeBinary>;

	struct Vector
	{
		static int GetSizeInBytes(GlobalSize Kglobal, LocalSize K); // returns -1 if invalid K's
		void Initialize(GlobalSize Kglobal, LocalSize K, NodeData data);  // called once when user adds a node
		void Add(GlobalSize Kglobal, LocalSize K, NodeData data); // called once when user calls MRFEnergy::AddNodeData()

		void SetZero(GlobalSize Kglobal, LocalSize K);                            // set this[k] = 0
		void Copy(GlobalSize Kglobal, LocalSize K, Vector* V);                    // set this[k] = V[k]
		void Add(GlobalSize Kglobal, LocalSize K, Vector* V);                     // set this[k] = this[k] + V[k]
		REAL GetValue(GlobalSize Kglobal, LocalSize K, Label k);                  // return this[k]
		REAL ComputeMin(GlobalSize Kglobal, LocalSize K, Label& kMin);            // return min_k { this[k] }, set kMin
		REAL ComputeAndSubtractMin(GlobalSize Kglobal, LocalSize K);              // same as previous, but additionally set this[k] -= vMin (and kMin is not returned)

		static int GetArraySize(GlobalSize Kglobal, LocalSize K);
		REAL GetArrayValue(GlobalSize Kglobal, LocalSize K, int k); // note: k is an integer in [0..GetArraySize()-1].
		                                                            // For Potts functions GetArrayValue() and GetValue() are the same,
		                                                            // but they are different for, say, 2-dimensional labels.
		void SetArrayValue(GlobalSize Kglobal, LocalSize K, int k, REAL x);

	private:
	friend struct Edge;
		REAL		m_data[2];
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
		// edge information
		REAL		m_lambdaIsing;

		// message
		Vector		m_message;
	};
};




//////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Implementation ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////



///////////////////// NodeData and EdgeData ///////////////////////

inline TypeBinary::NodeData::NodeData(REAL D0, REAL D1)
{
	m_data[0] = D0;
	m_data[1] = D1;
}

inline TypeBinary::EdgeData::EdgeData(REAL V00, REAL V01, REAL V10, REAL V11)
{
	m_V[0][0] = V00;
	m_V[0][1] = V01;
	m_V[1][0] = V10;
	m_V[1][1] = V11;
}

///////////////////// Vector ///////////////////////

inline int TypeBinary::Vector::GetSizeInBytes(GlobalSize Kglobal, LocalSize K)
{
	return sizeof(Vector);
}
inline void TypeBinary::Vector::Initialize(GlobalSize Kglobal, LocalSize K, NodeData data)
{
	m_data[0] = data.m_data[0];
	m_data[1] = data.m_data[1];
}

inline void TypeBinary::Vector::Add(GlobalSize Kglobal, LocalSize K, NodeData data)
{
	m_data[0] += data.m_data[0];
	m_data[1] += data.m_data[1];
}

inline void TypeBinary::Vector::SetZero(GlobalSize Kglobal, LocalSize K)
{
	m_data[0] = 0;
	m_data[1] = 0;
}

inline void TypeBinary::Vector::Copy(GlobalSize Kglobal, LocalSize K, Vector* V)
{
	m_data[0] = V->m_data[0];
	m_data[1] = V->m_data[1];
}

inline void TypeBinary::Vector::Add(GlobalSize Kglobal, LocalSize K, Vector* V)
{
	m_data[0] += V->m_data[0];
	m_data[1] += V->m_data[1];
}

inline TypeBinary::REAL TypeBinary::Vector::GetValue(GlobalSize Kglobal, LocalSize K, Label k)
{
	assert(k>=0 && k<2);
	return m_data[k];
}

inline TypeBinary::REAL TypeBinary::Vector::ComputeMin(GlobalSize Kglobal, LocalSize K, Label& kMin)
{
	kMin = (m_data[0] <= m_data[1]) ? 0 : 1;
	return m_data[kMin];
}

inline TypeBinary::REAL TypeBinary::Vector::ComputeAndSubtractMin(GlobalSize Kglobal, LocalSize K)
{
	REAL vMin;

	if (m_data[0] <= m_data[1])
	{
		vMin = m_data[0];
		m_data[0] = 0;
		m_data[1] -= vMin;
	}
	else
	{
		vMin = m_data[1];
		m_data[1] = 0;
		m_data[0] -= vMin;
	}

	return vMin;
}

inline int TypeBinary::Vector::GetArraySize(GlobalSize Kglobal, LocalSize K)
{
	return 2;
}

inline TypeBinary::REAL TypeBinary::Vector::GetArrayValue(GlobalSize Kglobal, LocalSize K, int k)
{
	assert(k>=0 && k<2);
	return m_data[k];
}

inline void TypeBinary::Vector::SetArrayValue(GlobalSize Kglobal, LocalSize K, int k, REAL x)
{
	assert(k>=0 && k<2);
	m_data[k] = x;
}

///////////////////// EdgeDataAndMessage implementation /////////////////////////

inline int TypeBinary::Edge::GetSizeInBytes(GlobalSize Kglobal, LocalSize Ki, LocalSize Kj, EdgeData data)
{
	return sizeof(Edge);
}

inline int TypeBinary::Edge::GetBufSizeInBytes(int vectorMaxSizeInBytes)
{
	return 0;
}

inline void TypeBinary::Edge::Initialize(GlobalSize Kglobal, LocalSize Ki, LocalSize Kj, EdgeData data, Vector* Di, Vector* Dj)
{
	// V00 V01  = A B  = A A  +  0.5 * ( 0   0   + 0 P-Q  +  0   P+Q  )
	// V10 V11    C D    D D             Q-P Q-P   0 P-Q     P+Q 0
    // where P=B-A, Q=C-D
	Di->m_data[0] += data.m_V[0][0];
	Di->m_data[1] += data.m_V[1][1];
	REAL P = data.m_V[0][1] - data.m_V[0][0], Q = data.m_V[1][0] - data.m_V[1][1];
	REAL halfPplusQ = (REAL)0.5 * (P + Q);
	REAL halfPminusQ = (REAL)0.5 * (P - Q);
	Di->m_data[1] += -halfPminusQ;
	Dj->m_data[1] += halfPminusQ;
	m_lambdaIsing = halfPplusQ;
	m_message.m_data[0] = 0;
	m_message.m_data[1] = halfPminusQ + (data.m_V[0][0] - data.m_V[1][1]);
}

inline TypeBinary::Vector* TypeBinary::Edge::GetMessagePtr()
{
	return &m_message;
}

inline void TypeBinary::Edge::Swap(GlobalSize Kglobal, LocalSize Ki, LocalSize Kj)
{
}

inline TypeBinary::REAL TypeBinary::Edge::UpdateMessage(GlobalSize Kglobal, LocalSize Ksource, LocalSize Kdest, Vector* source, REAL gamma, int dir, void* buf)
{
	REAL data[2], vMin;

	data[0] = gamma*source->m_data[0] - m_message.m_data[0];
	data[1] = gamma*source->m_data[1] - m_message.m_data[1];
	m_message.m_data[0] = (data[0] < data[1] + m_lambdaIsing) ? data[0] : data[1] + m_lambdaIsing;
	m_message.m_data[1] = (data[1] < data[0] + m_lambdaIsing) ? data[1] : data[0] + m_lambdaIsing;

	vMin = (m_message.m_data[0] < m_message.m_data[1]) ? m_message.m_data[0] : m_message.m_data[1];
	m_message.m_data[0] -= vMin;
	m_message.m_data[1] -= vMin;
	return vMin;
}




inline void TypeBinary::Edge::AddColumn(GlobalSize Kglobal, LocalSize Ksource, LocalSize Kdest, Label ksource, Vector* dest, int dir)
{
	dest->m_data[1-ksource] += m_lambdaIsing;
}

//////////////////////////////////////////////////////////////////////////////////

#endif
