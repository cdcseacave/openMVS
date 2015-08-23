/******************************************************************
typeBinaryFast.h

Same energy and interface as in typeBinary.h.
Faster implementation; however, values of lower bound and energy are computed
incorrectly. (Note that the solution is the same as with typeBinary.h).

For example usage, see typeBinary.h (only replace TypeBinary with TypeBinaryFast).
*******************************************************************/






#ifndef __TYPEBINARYFAST_H__
#define __TYPEBINARYFAST_H__

#include <string.h>
#include <assert.h>


template <class T> class MRFEnergy;


class TypeBinaryFast
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
friend class MRFEnergy<TypeBinaryFast>;

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
		REAL		m_data; // = D[1] - D[0]
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

inline TypeBinaryFast::NodeData::NodeData(REAL D0, REAL D1)
{
	m_data[0] = D0;
	m_data[1] = D1;
}

inline TypeBinaryFast::EdgeData::EdgeData(REAL V00, REAL V01, REAL V10, REAL V11)
{
	m_V[0][0] = V00;
	m_V[0][1] = V01;
	m_V[1][0] = V10;
	m_V[1][1] = V11;
}

///////////////////// Vector ///////////////////////

inline int TypeBinaryFast::Vector::GetSizeInBytes(GlobalSize Kglobal, LocalSize K)
{
	return sizeof(Vector);
}
inline void TypeBinaryFast::Vector::Initialize(GlobalSize Kglobal, LocalSize K, NodeData data)
{
	m_data = data.m_data[1] - data.m_data[0];
}

inline void TypeBinaryFast::Vector::Add(GlobalSize Kglobal, LocalSize K, NodeData data)
{
	m_data += data.m_data[1] - data.m_data[0];
}

inline void TypeBinaryFast::Vector::SetZero(GlobalSize Kglobal, LocalSize K)
{
	m_data = 0;
}

inline void TypeBinaryFast::Vector::Copy(GlobalSize Kglobal, LocalSize K, Vector* V)
{
	m_data = V->m_data;
}

inline void TypeBinaryFast::Vector::Add(GlobalSize Kglobal, LocalSize K, Vector* V)
{
	m_data += V->m_data;
}

inline TypeBinaryFast::REAL TypeBinaryFast::Vector::GetValue(GlobalSize Kglobal, LocalSize K, Label k)
{
	assert(k>=0 && k<2);
	return (k == 0) ? 0 : m_data;
}

inline TypeBinaryFast::REAL TypeBinaryFast::Vector::ComputeMin(GlobalSize Kglobal, LocalSize K, Label& kMin)
{
	kMin = (m_data >= 0) ? 0 : 1;
	return 0;
}

inline TypeBinaryFast::REAL TypeBinaryFast::Vector::ComputeAndSubtractMin(GlobalSize Kglobal, LocalSize K)
{
	return 0;
}

inline int TypeBinaryFast::Vector::GetArraySize(GlobalSize Kglobal, LocalSize K)
{
	return 1;
}

inline TypeBinaryFast::REAL TypeBinaryFast::Vector::GetArrayValue(GlobalSize Kglobal, LocalSize K, int k)
{
	assert(k==0);
	return m_data;
}

inline void TypeBinaryFast::Vector::SetArrayValue(GlobalSize Kglobal, LocalSize K, int k, REAL x)
{
	assert(k==0);
	m_data = x;
}

///////////////////// EdgeDataAndMessage implementation /////////////////////////

inline int TypeBinaryFast::Edge::GetSizeInBytes(GlobalSize Kglobal, LocalSize Ki, LocalSize Kj, EdgeData data)
{
	return sizeof(Edge);
}

inline int TypeBinaryFast::Edge::GetBufSizeInBytes(int vectorMaxSizeInBytes)
{
	return 0;
}

inline void TypeBinaryFast::Edge::Initialize(GlobalSize Kglobal, LocalSize Ki, LocalSize Kj, EdgeData data, Vector* Di, Vector* Dj)
{
	// V00 V01  = A B  = A A  +  0.5 * ( 0   0   + 0 P-Q  +  0   P+Q  )
	// V10 V11    C D    D D             Q-P Q-P   0 P-Q     P+Q 0
    // where P=B-A, Q=C-D
	Di->m_data += data.m_V[1][1] - data.m_V[0][0];
	REAL P = data.m_V[0][1] - data.m_V[0][0], Q = data.m_V[1][0] - data.m_V[1][1];
	REAL halfPplusQ = (REAL)0.5 * (P + Q);
	REAL halfPminusQ = (REAL)0.5 * (P - Q);
	Di->m_data += -halfPminusQ;
	Dj->m_data += halfPminusQ;
	m_lambdaIsing = halfPplusQ;
	m_message.m_data = halfPminusQ + (data.m_V[0][0] - data.m_V[1][1]);
}

inline TypeBinaryFast::Vector* TypeBinaryFast::Edge::GetMessagePtr()
{
	return &m_message;
}

inline void TypeBinaryFast::Edge::Swap(GlobalSize Kglobal, LocalSize Ki, LocalSize Kj)
{
}

inline TypeBinaryFast::REAL TypeBinaryFast::Edge::UpdateMessage(GlobalSize Kglobal, LocalSize Ksource, LocalSize Kdest, Vector* source, REAL gamma, int dir, void* buf)
{
	REAL s = gamma*source->m_data - m_message.m_data;

	if (m_lambdaIsing >= 0)
	{
		if      (s >= m_lambdaIsing)  m_message.m_data = m_lambdaIsing;
		else if (s <= -m_lambdaIsing) m_message.m_data = -m_lambdaIsing;
		else                          m_message.m_data = s;
	}
	else
	{
		if      (s >= -m_lambdaIsing) m_message.m_data = m_lambdaIsing;
		else if (s <= m_lambdaIsing)  m_message.m_data = -m_lambdaIsing;
		else                          m_message.m_data = -s;
	}

	return 0;
}

inline void TypeBinaryFast::Edge::AddColumn(GlobalSize Kglobal, LocalSize Ksource, LocalSize Kdest, Label ksource, Vector* dest, int dir)
{
	dest->m_data += (ksource == 0) ? m_lambdaIsing : -m_lambdaIsing;
}

//////////////////////////////////////////////////////////////////////////////////

#endif
