////////////////////////////////////////////////////////////////////
// Queue.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_QUEUE_H__
#define __SEACAVE_QUEUE_H__


// I N C L U D E S /////////////////////////////////////////////////



// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

/**************************************************************************************
// Queue template
**************************************************************************************/
template <class TYPE, class ARG_TYPE=const TYPE&, int useConstruct=1, int xGrow=16>
class cQueue
{
public:
	cQueue(int_t xSize = 16) :
		first(-1), last(-1), vectorSize(0), vector(NULL)
	{
		if (xSize > 0)
			Grow(xSize);
	}

	~cQueue()
	{
		Release();
	}

	void	Grow(int_t gr)
	{
		ASSERT(gr > 0);
		if (IsEmpty()) {
			ASSERT(first == -1 && last == -1);
			operator delete[] (vector);
			vectorSize += gr;
			vector = (TYPE*)  operator new[] (vectorSize * sizeof(TYPE));
			return;
		}
		TYPE* tmp = (TYPE*) operator new[] ((vectorSize + gr) * sizeof(TYPE));
		if (first > last) {
			memcpy(tmp, vector + first, (vectorSize - first) * sizeof(TYPE));
			memcpy(tmp + (vectorSize - first), vector, (last + 1) * sizeof(TYPE));
			last += vectorSize - first;
		} else {
			memcpy(tmp, vector + first, (last - first + 1) * sizeof(TYPE));
			last = last - first;
		}
		first = 0;
		vectorSize += gr;
		operator delete[] (vector);
		vector = tmp;
	}

	inline	TYPE*	GetData()
	{
		return vector;
	}

	inline void		PushHead()
	{
		_PushHead();
		if (useConstruct)
			new(vector+(--first)) TYPE();
		else
			--first;
	}
	inline void		PushTail()
	{
		_PushTail();
		if (useConstruct)
			new(vector+(++last)) TYPE();
		else
			++last;
	}

	inline TYPE&		AddEmptyHead()
	{
		_PushHead();
		if (useConstruct)
			return *(new(vector+(--first)) TYPE());
		else
			return vector[--first];
	}
	inline TYPE&		AddEmptyTail()
	{
		_PushTail();
		if (useConstruct)
			return *(new(vector+(++last)) TYPE());
		else
			return vector[++last];
	}

	inline void		AddHead(ARG_TYPE elem)
	{
		_PushHead();
		if (useConstruct)
			new(vector+(--first)) TYPE(elem);
		else
			vector[--first] = elem;
	}
	inline void		AddTail(ARG_TYPE elem)
	{
		_PushTail();
		if (useConstruct)
			new(vector+(++last)) TYPE(elem);
		else
			vector[++last] = elem;
	}

	inline	TYPE&	GetHead() const
	{
		ASSERT(first >= 0 && last >= 0);
		return vector[first];
	}
	inline	TYPE&	GetTail() const
	{
		ASSERT(first >= 0 && last >= 0);
		return vector[last];
	}

	inline	TYPE&	GetHead(uint_t delta) const
	{
		ASSERT(first >= 0 && last >= 0 && delta < GetSize());
		delta += first;
		if ((int_t)delta < vectorSize)
			return vector[delta];
		return vector[delta-vectorSize];
	}
	inline	TYPE&	GetTail(uint_t delta) const
	{
		ASSERT(first >= 0 && last >= 0 && delta < GetSize());
		if ((int_t)delta <= last)
			return vector[last-delta];
		return vector[vectorSize-(delta-last)];
	}

	inline void		PopHead()
	{
		ASSERT(first >= 0 && last >= 0);
		if (useConstruct)
			(vector+first)->~TYPE();
		if (first > last) {
			if (++first >= vectorSize)
				first = 0;
		} else {
			if (++first > last)
				first = last = -1;
		}
	}
	inline void		PopTail()
	{
		ASSERT(first >= 0 && last >= 0);
		if (useConstruct)
			(vector+last)->~TYPE();
		if (last >= first) {
			if (--last < first)
				first = last = -1;
		} else {
			if (--last < 0)
				last = vectorSize-1;
		}
	}

	inline TYPE		RemoveHead()
	{
		ASSERT(first >= 0 && last >= 0);
		if (useConstruct) {
			TYPE elem(vector[first]);
			PopHead();
			return elem;
		} else {
			TYPE& elem(vector[first]);
			PopHead();
			return elem;
		}
	}
	inline TYPE		RemoveTail()
	{
		ASSERT(first >= 0 && last >= 0);
		if (useConstruct) {
			TYPE elem(vector[last]);
			PopTail();
			return elem;
		} else {
			TYPE& elem(vector[last]);
			PopTail();
			return elem;
		}
	}

	inline	uint_t	GetSize() const
	{
		if (first < 0)
			return 0;
		if (first > last) // circular
			return (vectorSize - (first - last) + 1);
		return uint_t(last - first + 1);
	}

	inline	void	Empty()
	{
		first = last = -1;
	}

	// same as Empty(), plus free all allocated memory
	inline	void	Release()
	{
		ASSERT((vector != NULL && vectorSize > 0) || (vector == NULL && vectorSize == 0));
		if (vector != NULL) {
			if (useConstruct) {
				if (first > last) {
					for (; first < vectorSize; first++)
						(vector+first)->~TYPE();
					for (; last >= 0; last--)
						(vector+last)->~TYPE();
				} else if (first >= 0) {
					for (; first <= last; first++)
						(vector+first)->~TYPE();
				}
			}
			operator delete[] (vector);
			vector = NULL;
			vectorSize = 0;
		}
		Empty();
	}

	inline	bool	IsEmpty() const
	{
		return (first < 0);
	}

	inline	TYPE&	operator[](uint_t index) const
	{
		return GetHead(index);
	}

protected:
	inline void		_PushHead()
	{
		if (first > last) {
			if (last + 1 == first) {
				Grow(xGrow);
				first = vectorSize;
			}
		} else {
			if (first <= 0) {
				if (last + 1 >= vectorSize)
					Grow(xGrow);
				first = vectorSize;
			}
			if (last < 0)
				last = first - 1;
		}
	}
	inline void		_PushTail()
	{
		if (last >= first) {
			if (last + 1 >= vectorSize) {
				if (first <= 0)
					Grow(xGrow);
				else
					last = -1;
			}
			if (first < 0)
				first = 0;
		} else {
			if (last + 1 == first)
				Grow(xGrow);
		}
	}

protected:
	int_t	first;
	int_t	last;
	int_t	vectorSize;
	TYPE*	vector;
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_QUEUE_H__
