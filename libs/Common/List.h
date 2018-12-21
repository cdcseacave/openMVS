////////////////////////////////////////////////////////////////////
// List.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_LIST_H__
#define __SEACAVE_LIST_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////

#ifndef _USE_VECTORINTERFACE
#define _USE_VECTORINTERFACE 1
#endif

// cList index type
#ifdef _SUPPORT_CPP11
#ifdef _MSC_VER
#define ARR2IDX(arr) std::remove_reference<decltype(arr)>::type::IDX
#else
#define ARR2IDX(arr) typename std::remove_reference<decltype(arr)>::type::IDX
#endif
#else
#define ARR2IDX(arr) IDX
#endif

// cList iterator by index
#ifndef FOREACH
#define FOREACH(var, arr) for (ARR2IDX(arr) var=0, var##Size=(arr).GetSize(); var<var##Size; ++var)
#endif
#ifndef RFOREACH
#define RFOREACH(var, arr) for (ARR2IDX(arr) var=(arr).GetSize(); var-->0; )
#endif
// cList iterator by pointer
#ifndef FOREACHPTR
#define FOREACHPTR(var, arr) for (auto var=(arr).Begin(), var##End=(arr).End(); var!=var##End; ++var)
#endif
#ifndef RFOREACHPTR
#define RFOREACHPTR(var, arr) for (auto var=(arr).End(), var##Begin=(arr).Begin(); var--!=var##Begin; )
#endif

// raw data array iterator by index
#ifndef FOREACHRAW
#define FOREACHRAW(var, sz) for (IDX var=0, var##Size=(sz); var<var##Size; ++var)
#endif
#ifndef RFOREACHRAW
#define RFOREACHRAW(var, sz) for (IDX var=sz; var-->0; )
#endif
// raw data array iterator by pointer
#ifndef FOREACHRAWPTR
#define FOREACHRAWPTR(var, arr, sz) for (auto var=(arr), var##End=var+sz; var!=var##End; ++var)
#endif
#ifndef RFOREACHRAWPTR
#define RFOREACHRAWPTR(var, arr, sz) for (auto var##Begin=(arr), var=var##Begin+sz; var--!=var##Begin; )
#endif

// constructs a cList reference to a given raw data array
#ifndef CLISTREFRAW
#define CLISTREFRAW(CLIST, var, arr, sz) uint8_t _ArrData##var[sizeof(CLIST)]; new(_ArrData##var) CLIST(sz, arr); const CLIST& var(*((const CLIST*)_ArrData##var))
#endif
// constructs a cList reference to a given std::_vector
#ifndef CLISTREFVECTOR
#define CLISTREFVECTOR(CLIST, var, vec) uint8_t _ArrData##var[sizeof(CLIST)]; new(_ArrData##var) CLIST(vec.size(), &vec[0]); const CLIST& var(*((const CLIST*)_ArrData##var))
#endif

#define CLISTDEF0(TYPE) SEACAVE::cList< TYPE, const TYPE&, 0 >
#define CLISTDEF2(TYPE) SEACAVE::cList< TYPE, const TYPE&, 2 >
#define CLISTDEFIDX(TYPE,IDXTYPE) SEACAVE::cList< TYPE, const TYPE&, 1, 16, IDXTYPE >
#define CLISTDEF0IDX(TYPE,IDXTYPE) SEACAVE::cList< TYPE, const TYPE&, 0, 16, IDXTYPE >


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

typedef size_t IDX;
#define NO_IDX DECLARE_NO_INDEX(IDX)

/**************************************************************************************
 * List template
 * --------------
 * fast array
 * - set "useConstruct" to 0 for not using constructors/destructors
 *   when adding/removing elements to the array
 * - set "useConstruct" to 1 for using memcpy/memmove when moving arrays
 *   instead of copy-constructors
 * - set "useConstruct" to 2 for always using constructors/destructors
 * - set "grow" to the number of elements to increase the allocated array
 *   when more space is needed
 * - if using sorting, the operator < needs to be implemented (the
 *   operator == is also needed if argument is not ARG_TYPE)
 **************************************************************************************/

template <
	typename TYPE,
	typename ARG_TYPE=const TYPE&,
	int useConstruct=1,
	int grow=16,
	typename IDX_TYPE=IDX>
class cList
{
public:
	typedef TYPE Type;
	typedef ARG_TYPE ArgType;
	typedef IDX_TYPE IDX;
	typedef int (STCALL *TFncCompare)(const void* elem, const void* key); //returns 0 if equal, otherwise <0 or >0 respectively
	typedef bool (STCALL *TFncCompareBool)(ARG_TYPE elem, ARG_TYPE key); //returns true if the two elements are strict in weak ordering

	// construct an empty list
	inline cList() : _size(0), _vectorSize(0), _vector(NULL)
	{
	}

	// construct a list containing size initialized elements
	cList(IDX size) : _size(size), _vectorSize(size), _vector((TYPE*)operator new[] (size * sizeof(TYPE)))
	{
		ASSERT(size > 0 && size < NO_INDEX);
		_ArrayConstruct(_vector, size);
	}

	// construct a list containing size initialized elements and allocated space for _reserved elements
	cList(IDX size, IDX _reserved) : _size(size), _vectorSize(_reserved), _vector((TYPE*)operator new[] (_reserved * sizeof(TYPE)))
	{
		ASSERT(_reserved >= size && _reserved < NO_INDEX);
		_ArrayConstruct(_vector, size);
	}

	// copy constructor: creates a deep-copy of the given list
	cList(const cList& rList) : _size(rList._size), _vectorSize(rList._vectorSize), _vector(NULL)
	{
		if (_vectorSize == 0) {
			ASSERT(_size == 0);
			return;
		}
		_vector = (TYPE*)(operator new[] (_vectorSize * sizeof(TYPE)));
		_ArrayCopyConstruct(_vector, rList._vector, _size);
	}

	// constructor a list from a raw data array, taking ownership of the array memory
	explicit inline cList(IDX nSize, TYPE* pData) : _size(nSize), _vectorSize(nSize), _vector(pData)
	{
	}

	inline ~cList()
	{
		_Release();
	}

	// copy the content from the given list
	inline	cList&	operator=(const cList& rList)
	{
		return CopyOf(rList);
	}

	inline	cList&	CopyOf(const cList& rList, bool bForceResize=false)
	{
		if (this == &rList)
			return (*this);
		if (bForceResize || _vectorSize < rList._vectorSize) {
			_Release();
			_vectorSize = rList._vectorSize;
			_vector = (TYPE*) operator new[] (_vectorSize * sizeof(TYPE));
			_ArrayCopyConstruct(_vector, rList._vector, rList._size);
		} else {
			if (_size >= rList._size) {
				_ArrayDestruct(_vector+rList._size, _size-rList._size);
				_ArrayCopyRestrict(_vector, rList._vector, rList._size);
			} else {
				_ArrayCopyRestrict(_vector, rList._vector, _size);
				_ArrayCopyConstruct(_vector+_size, rList._vector+_size, rList._size-_size);
			}
		}
		_size = rList._size;
		return (*this);
	}

	inline	cList&	CopyOf(const TYPE* pData, IDX nSize, bool bForceResize=false)
	{
		if (_vector == pData)
			return (*this);
		if (bForceResize || _vectorSize < nSize) {
			_Release();
			_vectorSize = nSize;
			_vector = (TYPE*) operator new[] (_vectorSize * sizeof(TYPE));
			_ArrayCopyConstruct(_vector, pData, nSize);
		} else {
			if (_size >= nSize) {
				_ArrayDestruct(_vector+nSize, _size-nSize);
				_ArrayCopyRestrict(_vector, pData, nSize);
			} else {
				_ArrayCopyRestrict(_vector, pData, _size);
				_ArrayCopyConstruct(_vector+_size, pData+_size, nSize-_size);
			}
		}
		_size = nSize;
		return (*this);
	}

	// release current list and swap the content with the given list
	inline	cList&	CopyOfRemove(cList& rList)
	{
		if (this == &rList)
			return (*this);
		_Release();
		_size = rList._size;
		_vectorSize = rList._vectorSize;
		_vector = rList._vector;
		rList._vector = NULL;
		rList._size = rList._vectorSize = 0;
		return (*this);
	}

	inline	void	Join(const cList& rList)
	{
		if (this == &rList || rList._size == 0)
			return;
		const IDX newSize = _size + rList._size;
		Reserve(newSize);
		_ArrayCopyConstruct(_vector+_size, rList._vector, rList._size);
		_size = newSize;
	}
	inline	void	Join(const TYPE* pData, IDX nSize)
	{
		const IDX newSize = _size + nSize;
		Reserve(newSize);
		_ArrayCopyConstruct(_vector+_size, pData, nSize);
		_size = newSize;
	}

	inline	void	JoinRemove(cList& rList)
	{
		if (this == &rList || rList._size == 0)
			return;
		const IDX newSize(_size + rList._size);
		Reserve(newSize);
		_ArrayMoveConstruct<true>(_vector+_size, rList._vector, rList._size);
		_size = newSize;
		rList._size = 0;
	}

	// Swap the elements of the two lists.
	inline	void	Swap(cList& rList)
	{
		if (this == &rList)
			return;
		const IDX tmpSize = _size;
		_size = rList._size;
		rList._size = tmpSize;
		const IDX tmpVectorSize = _vectorSize;
		_vectorSize = rList._vectorSize;
		rList._vectorSize = tmpVectorSize;
		TYPE* const tmpVector = _vector;
		_vector = rList._vector;
		rList._vector = tmpVector;
	}

	// Swap the two elements.
	inline	void	Swap(IDX idx1, IDX idx2)
	{
		ASSERT(idx1 < _size && idx2 < _size);
		TYPE tmp = _vector[idx1];
		_vector[idx1] = _vector[idx2];
		_vector[idx2] = tmp;
	}

	// Set the allocated memory (normally used for types without constructor).
	inline void		Memset(uint8_t val)
	{
		memset(_vector, val, _size * sizeof(TYPE));
	}
	inline void		MemsetValue(ARG_TYPE val)
	{
		std::fill_n(_vector, _size, val);
	}

	// Delete the old array, and create a new one of the desired size;
	// the old elements are deleted and the array is filled with new empty ones.
	inline void		Reset(IDX newSize)
	{
		_Release();
		_vectorSize = newSize;
		_size = newSize;
		if (newSize == 0) {
			_vector = NULL;
			return;
		}
		_vector = (TYPE*) operator new[] (newSize * sizeof(TYPE));
		_ArrayConstruct(_vector, newSize);
	}

	// Pre-allocate memory for the array at the desired size;
	// the old elements are kept.
	inline	void	Reserve(IDX needVectorSize)
	{
		if (_vectorSize < needVectorSize)
			_Grow(needVectorSize);
	}
	// Same as above, but only the extra needed space is passed.
	inline	void	ReserveExtra(IDX needVectorExtraSize)
	{
		if (_vectorSize < _size+needVectorExtraSize)
			_Grow(_vectorSize+MAXF(needVectorExtraSize,(IDX)grow));
	}

	// Set the size of the array at the new desired size;
	// the old elements are kept, but only the ones that are below the new size.
	// If increased, the array is filled with new empty elements.
	inline	void	Resize(IDX newSize)
	{
		if (newSize == _size)
			return;
		if (newSize < _size)
			return _Shrink(newSize);
		Reserve(newSize);
		_ArrayConstruct(_vector+_size, newSize-_size);
		_size = newSize;
	}
	inline	void	ResizeExact(IDX newSize)
	{
		if (newSize == _size)
			return;
		if (newSize < _size)
			return _ShrinkExact(newSize);
		Reserve(newSize);
		_ArrayConstruct(_vector+_size, newSize-_size);
		_size = newSize;
	}

	// Free unused memory.
	inline void		ReleaseFree()
	{
		if (_size < _vectorSize)
			_ShrinkExact(_size);
	}

	inline IDX		GetIndex(const TYPE* pElem) const
	{
		ASSERT(pElem-_vector < _size);
		return (IDX)(pElem-_vector);
	}

	inline const TYPE&	operator[](IDX index) const
	{
		ASSERT(index < _size);
		return _vector[index];
	}
	inline TYPE&	operator[](IDX index)
	{
		ASSERT(index < _size);
		return _vector[index];
	}

	inline TYPE*	GetData() const
	{
		return _vector;
	}
	inline size_t	GetDataSize() const
	{
		return sizeof(TYPE)*_size;
	}

	inline TYPE*	Begin() const
	{
		return _vector;
	}

	inline TYPE*	End() const
	{
		return _vector+_size;
	}

	inline const TYPE&	First() const
	{
		ASSERT(_size > 0);
		return _vector[0];
	}
	inline TYPE&	First()
	{
		ASSERT(_size > 0);
		return _vector[0];
	}

	inline const TYPE&	Last() const
	{
		ASSERT(_size > 0);
		return _vector[_size-1];
	}
	inline TYPE&	Last()
	{
		ASSERT(_size > 0);
		return _vector[_size-1];
	}

	// Adds a new empty element at the end of the array.
	inline TYPE&	AddEmpty()
	{
		if (_vectorSize <= _size)
			_Grow(_vectorSize + grow);
		if (useConstruct)
			return *(new(_vector + (_size++)) TYPE);
		else
			return _vector[_size++];
	}
	inline TYPE*	AddEmpty(IDX numElems)
	{
		const IDX newSize = _size + numElems;
		if (_vectorSize < newSize)
			_Grow(newSize + grow);
		Type* const pData = _vector + _size;
		_ArrayConstruct(pData, numElems);
		_size = newSize;
		return pData;
	}

	// Adds a new empty element at the end of the array and pass the arguments to its constructor.
	#ifdef _SUPPORT_CPP11
	template <typename... Args>
	inline TYPE&	AddConstruct(Args&&... args)
	{
		if (_vectorSize <= _size)
			_Grow(_vectorSize + grow);
		return *(new(_vector + (_size++)) TYPE(std::forward<Args>(args)...));
	}
	#else
	template <typename... Args>
	inline TYPE&	AddConstruct(Args... args)
	{
		if (_vectorSize <= _size)
			_Grow(_vectorSize + grow);
		return *(new(_vector + (_size++)) TYPE(args...));
	}
	#endif

	inline IDX		InsertEmpty()
	{
		if (_vectorSize <= _size)
			_Grow(_vectorSize + grow);
		if (useConstruct)
			new(_vector + _size) TYPE;
		return _size++;
	}

	// Adds the new element at the end of the array.
	#ifdef _SUPPORT_CPP11
	template <typename T>
	inline void		Insert(T&& elem)
	{
		if (_vectorSize <= _size)
			_Grow(_vectorSize + grow);
		if (useConstruct)
			new(_vector+(_size++)) TYPE(std::forward<T>(elem));
		else
			_vector[_size++] = std::forward<T>(elem);
	}
	#else
	inline void		Insert(ARG_TYPE elem)
	{
		if (_vectorSize <= _size)
			_Grow(_vectorSize + grow);
		if (useConstruct)
			new(_vector+(_size++)) TYPE(elem);
		else
			_vector[_size++] = elem;
	}
	#endif

	#ifdef _SUPPORT_CPP11
	template <typename T>
	inline void		SetAt(IDX index, T&& elem)
	{
		if (_size <= index)
			Resize(index + 1);
		_vector[index] = std::forward<T>(elem);
	}
	#else
	inline void		SetAt(IDX index, ARG_TYPE elem)
	{
		if (_size <= index)
			Resize(index + 1);
		_vector[index] = elem;
	}
	#endif

	#ifdef _SUPPORT_CPP11
	template <typename T>
	inline void		AddAt(IDX index, T&& elem)
	{
		if (index < _size)
			return InsertAt(index, std::forward<T>(elem));
		const IDX newSize = index + 1;
		if (_vectorSize <= newSize)
			_Grow(newSize + grow);
		_ArrayConstruct(_vector+_size, index-_size);
		if (useConstruct)
			new(_vector+index) TYPE(std::forward<T>(elem));
		else
			_vector[index] = std::forward<T>(elem);
		++_size;
	}
	#else
	inline void		AddAt(IDX index, ARG_TYPE elem)
	{
		if (index < _size)
			return InsertAt(index, elem);
		const IDX newSize = index + 1;
		if (_vectorSize <= newSize)
			_Grow(newSize + grow);
		_ArrayConstruct(_vector+_size, index-_size);
		if (useConstruct)
			new(_vector+index) TYPE(elem);
		else
			_vector[index] = elem;
		++_size;
	}
	#endif

	#ifdef _SUPPORT_CPP11
	template <typename T>
	inline void		InsertAt(IDX index, T&& elem)
	{
		if (useConstruct)
			new(AllocateAt(index)) TYPE(std::forward<T>(elem));
		else
			*AllocateAt(index) = std::forward<T>(elem);
	}
	#else
	inline void		InsertAt(IDX index, ARG_TYPE elem)
	{
		if (useConstruct)
			new(AllocateAt(index)) TYPE(elem);
		else
			*AllocateAt(index) = elem;
	}
	#endif

	// Same as Insert, but the constructor is not called.
	inline TYPE*	Allocate()
	{
		if (_vectorSize <= _size)
			_Grow(_vectorSize + grow);
		return _vector+_size++;
	}
	inline TYPE*	AllocateAt(IDX index)
	{
		ASSERT(index <= _size);
		const IDX move(_size-index);
		Allocate();
		_ArrayMoveConstruct<false>(_vector+index+1, _vector+index, move);
		return _vector+index;
	}

	inline IDX		InsertSort(ARG_TYPE elem)
	{
		IDX l1(0), l2(_size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE compElem(_vector[i]);
			if (elem < compElem)
				l2 = i;
			else if (compElem < elem)
				l1 = i+1;
			else {
				InsertAt(i, elem);
				return i;
			}
		}
		InsertAt(l1, elem);
		return l1;
	}

	inline IDX		InsertSortPtr(ARG_TYPE elem)
	{
		IDX l1(0), l2(_size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE compElem(_vector[i]);
			if (*elem < *compElem)
				l2 = i;
			else if (*compElem < *elem)
				l1 = i+1;
			else {
				InsertAt(i, elem);
				return i;
			}
		}
		InsertAt(l1, elem);
		return l1;
	}

	inline IDX		InsertSort(ARG_TYPE elem, TFncCompare xCompare)
	{
		IDX l1(0), l2(_size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			const int res(xCompare(_vector+i, &elem));
			if (res == 0) {
				InsertAt(i, elem);
				return i;
			}
			if (res < 0)
				l1 = i+1;
			else
				l2 = i;
		}
		InsertAt(l1, elem);
		return l1;
	}

	inline TYPE&	GetNth(IDX index)
	{
		TYPE* const nth(Begin()+index);
		std::nth_element(Begin(), nth, End());
		return *nth;
	}
	inline TYPE&	GetMedian()
	{
		return GetNth(_size >> 1);
	}

	inline TYPE		GetMean()
	{
		return std::accumulate(Begin(), End(), TYPE(0)) / _size;
	}

	inline void		Sort()
	{
		std::sort(Begin(), End());
	}
	inline void		Sort(TFncCompareBool xCompare)
	{
		std::sort(Begin(), End(), xCompare);
	}

	inline	bool	IsSorted() const
	{
		if (_size < 2)
			return true;
		IDX i = _size-1;
		do {
			ARG_TYPE elem1 = _vector[i];
			ARG_TYPE elem0 = _vector[--i];
			if (!(elem0 < elem1 || elem0 == elem1))
				return false;
		} while (i > 0);
		return true;
	}
	inline	bool	IsSorted(TFncCompareBool xCompare) const
	{
		if (_size < 2)
			return true;
		IDX i = _size-1;
		do {
			ARG_TYPE elem1 = _vector[i];
			ARG_TYPE elem0 = _vector[--i];
			if (!xCompare(elem0, elem1))
				return false;
		} while (i > 0);
		return true;
	}

	inline std::pair<IDX,bool>	InsertSortUnique(ARG_TYPE elem)
	{
		IDX l1(0), l2(_size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE compElem(_vector[i]);
			if (elem < compElem)
				l2 = i;
			else if (compElem < elem)
				l1 = i+1;
			else
				return std::make_pair(i,true);
		}
		InsertAt(l1, elem);
		return std::make_pair(l1, false);
	}

	inline std::pair<IDX,bool>	InsertSortUniquePtr(ARG_TYPE elem)
	{
		IDX l1(0), l2(_size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE compElem(_vector[i]);
			if (*elem < *compElem)
				l2 = i;
			else if (*compElem < *elem)
				l1 = i+1;
			else
				return std::make_pair(i, true);
		}
		InsertAt(l1, elem);
		return std::make_pair(l1, false);
	}

	inline std::pair<IDX,bool>	InsertSortUnique(ARG_TYPE elem, TFncCompare xCompare)
	{
		IDX l1(0), l2(_size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			const int res(xCompare(_vector+i, &elem));
			if (res == 0)
				return std::make_pair(i, true);
			if (res < 0)
				l1 = i + 1;
			else
				l2 = i;
		}
		InsertAt(l1, elem);
		return std::make_pair(l1, false);
	}

	// insert given element if it is in the top N
	template <int N>
	inline void		StoreTop(ARG_TYPE elem) {
		const IDX idx(FindFirstEqlGreater(elem));
		if (idx < _size) {
			if (_size >= N)
				RemoveLast();
			InsertAt(idx, elem);
		} else if (_size < N) {
			Insert(elem);
		}
	}
	inline void		StoreTop(ARG_TYPE elem, IDX N) {
		const IDX idx(FindFirstEqlGreater(elem));
		if (idx < _size) {
			if (_size >= N)
				RemoveLast();
			InsertAt(idx, elem);
		} else if (_size < N) {
			Insert(elem);
		}
	}

	inline IDX		FindFirst(ARG_TYPE searchedKey) const
	{
		IDX l1(0), l2(_size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(_vector[i]);
			if (searchedKey < key)
				l2 = i;
			else if (key < searchedKey)
				l1 = i + 1;
			else
				return i;
		}
		return NO_INDEX;
	}

	inline IDX		FindFirstPtr(ARG_TYPE searchedKey) const
	{
		IDX l1(0), l2(_size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(_vector[i]);
			if (*searchedKey < *key)
				l2 = i;
			else if (*key < *searchedKey)
				l1 = i + 1;
			else
				return i;
		}
		return NO_INDEX;
	}

	template <typename SEARCH_TYPE>
	inline IDX		FindFirst(const SEARCH_TYPE& searchedKey) const
	{
		IDX l1(0), l2(_size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(_vector[i]);
			if (key == searchedKey)
				return i;
			if (key < searchedKey)
				l1 = i + 1;
			else
				l2 = i;
		}
		return NO_INDEX;
	}

	template <typename SEARCH_TYPE>
	inline IDX		FindFirstPtr(const SEARCH_TYPE& searchedKey) const
	{
		IDX l1(0), l2(_size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(_vector[i]);
			if (*key == searchedKey)
				return i;
			if (*key < searchedKey)
				l1 = i + 1;
			else
				l2 = i;
		}
		return NO_INDEX;
	}

	inline IDX		FindFirst(const void* searchedKey, TFncCompare xCompare) const
	{
		IDX l1(0), l2(_size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			const int res(xCompare(_vector+i, searchedKey));
			if (res == 0)
				return i;
			if (res < 0)
				l1 = i + 1;
			else
				l2 = i;
		}
		return NO_INDEX;
	}

	inline IDX		FindFirstBelow(ARG_TYPE searchedKey) const
	{
		if (_size == 0) return NO_INDEX;
		IDX l1(0), l2(_size);
		do {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(_vector[i]);
			if (searchedKey < key)
				l2 = i;
			else if (key < searchedKey)
				l1 = i+1;
			else {
				while (i-- && !(_vector[i] < searchedKey));
				return i;
			}
		} while (l1 < l2);
		return l1-1;
	}

	template <typename SEARCH_TYPE>
	inline IDX		FindFirstBelow(const SEARCH_TYPE& searchedKey) const
	{
		if (_size == 0) return NO_INDEX;
		IDX l1(0), l2(_size);
		do {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(_vector[i]);
			if (key == searchedKey) {
				while (i-- && _vector[i] == searchedKey);
				return i;
			}
			if (key < searchedKey)
				l1 = i+1;
			else
				l2 = i;
		} while (l1 < l2);
		return l1-1;
	}

	inline IDX		FindFirstEqlGreater(ARG_TYPE searchedKey) const
	{
		if (_size == 0) return 0;
		IDX l1(0), l2(_size);
		do {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(_vector[i]);
			if (searchedKey < key)
				l2 = i;
			else if (key < searchedKey)
				l1 = i+1;
			else {
				while (i-- && !(_vector[i] < searchedKey));
				return i+1;
			}
		} while (l1 < l2);
		return l1;
	}

	template <typename SEARCH_TYPE>
	inline IDX		FindFirstEqlGreater(const SEARCH_TYPE& searchedKey) const
	{
		if (_size == 0) return 0;
		IDX l1(0), l2(_size);
		do {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(_vector[i]);
			if (key == searchedKey) {
				while (i-- && _vector[i] == searchedKey);
				return i+1;
			}
			if (key < searchedKey)
				l1 = i+1;
			else
				l2 = i;
		} while (l1 < l2);
		return l1;
	}

	// find the matching "elem" by brute-force (front to back)
	// returns the first element found
	inline IDX		Find(ARG_TYPE elem) const
	{
		for (IDX i = 0; i < _size; ++i)
			if (_vector[i] == elem)
				return i;
		return NO_INDEX;
	}
	template <typename SEARCH_TYPE>
	inline IDX		Find(const SEARCH_TYPE& searchedKey) const
	{
		for (IDX i = 0; i < _size; ++i)
			if (_vector[i] == searchedKey)
				return i;
		return NO_INDEX;
	}
	template <typename Functor>
	inline IDX		FindFunc(const Functor& functor) const
	{
		for (IDX i = 0; i < _size; ++i)
			if (functor(_vector[i]))
				return i;
		return NO_INDEX;
	}

	// find the matching "elem" by brute-force (back to front)
	// returns the first element found
	inline IDX		RFind(ARG_TYPE elem) const
	{
		IDX i(_size);
		while (i)
			if (_vector[--i] == elem)
				return i;
		return NO_INDEX;
	}
	template <typename SEARCH_TYPE>
	inline IDX		RFind(const SEARCH_TYPE& searchedKey) const
	{
		IDX i(_size);
		while (i)
			if (_vector[--i] == searchedKey)
				return i;
		return NO_INDEX;
	}
	template <typename Functor>
	inline IDX		RFindFunc(const Functor& functor) const
	{
		IDX i(_size);
		while (i)
			if (functor(_vector[--i]))
				return i;
		return NO_INDEX;
	}

	// call a function or lambda on each element
	template <typename Functor>
	inline	void	ForEach(const Functor& functor) const
	{
		FOREACH(i, *this)
			functor(i);
	}
	template <typename Functor>
	inline	void	ForEachPtr(const Functor& functor) const
	{
		FOREACHPTR(ptr, *this)
			functor(ptr);
	}
	template <typename Functor>
	inline	void	ForEachRef(const Functor& functor) const
	{
		FOREACHPTR(ptr, *this)
			functor(*ptr);
	}
	// same, but in reverse order
	template <typename Functor>
	inline	void	RForEach(const Functor& functor) const
	{
		RFOREACH(i, *this)
			functor(i);
	}
	template <typename Functor>
	inline	void	RForEachPtr(const Functor& functor) const
	{
		RFOREACHPTR(ptr, *this)
			functor(ptr);
	}
	template <typename Functor>
	inline	void	RForEachRef(const Functor& functor) const
	{
		RFOREACHPTR(ptr, *this)
			functor(*ptr);
	}

	// Erase each element matching "elem".
	inline void		Remove(ARG_TYPE elem)
	{
		IDX i = _size;
		while (i)
			if (_vector[--i] == elem)
				RemoveAt(i);
	}

	inline ARG_TYPE	RemoveTail()
	{
		ASSERT(_size);
		ASSERT(!useConstruct);
		return _vector[--_size];
	}

	inline void		RemoveLast()
	{
		ASSERT(_size);
		_ArrayDestruct(_vector+(--_size), 1);
	}

	inline void		RemoveLast(IDX count)
	{
		ASSERT(count <= _size);
		_ArrayDestruct(_vector+(_size-=count), count);
	}

	inline void		RemoveAt(IDX index)
	{
		ASSERT(index < _size);
		if (index+1 == _size)
			RemoveLast();
		else
			_ArrayMoveCopy<true>(_vector+index, _vector+(--_size), 1);
	}

	inline void		RemoveAt(IDX index, IDX count)
	{
		ASSERT(index+count <= _size);
		if (index+count == _size) {
			RemoveLast(count);
		} else {
			_size -= count;
			const IDX move(_size-index);
			TYPE* const vectorEnd(_vector+_size);
			if (move < count) {
				const IDX del(count-move);
				_ArrayMoveCopy<true>(_vector+index, vectorEnd+del, move);
				_ArrayDestruct(vectorEnd, del);
			} else {
				_ArrayMoveCopy<false>(_vector+index, vectorEnd, count);
			}
		}
	}

	inline void		RemoveAtMove(IDX index)
	{
		ASSERT(index < _size);
		if (index+1 == _size) {
			RemoveLast();
		} else {
			_ArrayDestruct(_vector+index, 1);
			_ArrayMoveConstructFwd(_vector+index, _vector+index+1, (--_size)-index);
		}
	}

	inline void		RemoveAtMove(IDX index, IDX count)
	{
		ASSERT(index+count <= _size);
		if (index+count == _size) {
			RemoveLast(count);
		} else {
			_ArrayDestruct(_vector+index, count);
			_ArrayMoveConstructFwd(_vector+index, _vector+index+count, (_size-=count)-index);
		}
	}

	inline	void	Empty()
	{
		_ArrayDestruct(_vector, _size);
		_size = 0;
	}

	// same as Empty(), plus free all allocated memory
	inline	void	Release()
	{
		_Release();
		_Init();
	}

	// Discard all stored data and initialize it as an empty array.
	// (note: call this only when you know what you are doing,
	// you have to deallocate yourself all elements and _vector data)
	inline void		Reset()
	{
		_Init();
	}

	// Delete also the pointers (take care to use this function only if the elements are pointers).
	inline void		EmptyDelete()
	{
		while (_size)
			delete _vector[--_size];
	}

	// same as EmptyDelete(), plus free all allocated memory
	inline void		ReleaseDelete()
	{
		EmptyDelete();
		operator delete[] (_vector);
		_vector = NULL;
		_vectorSize = 0;
	}

	inline	IDX		GetCapacity() const
	{
		return _vectorSize;
	}

	inline	IDX		GetSize() const
	{
		return _size;
	}

	inline	bool	IsEmpty() const
	{
		return (_size == 0);
	}

	static inline unsigned GetConstructType()
	{
		return useConstruct;
	}
	static inline IDX GetGrowSize()
	{
		return grow;
	}

protected:
	// Free all memory.
	inline	void	_Release()
	{
		_ArrayDestruct(_vector, _size);
		operator delete[] (_vector);
	}

	// Initialize array.
	inline	void	_Init()
	{
		_vector = NULL;
		_vectorSize = _size = 0;
	}

	// Increase the size of the array at least to the specified amount.
	inline void		_Grow(IDX newVectorSize)
	{
		ASSERT(newVectorSize > _vectorSize);
		// grow by 50% or at least to minNewVectorSize
		const IDX expoVectorSize(_vectorSize + (_vectorSize>>1));
		if (newVectorSize < expoVectorSize)
			newVectorSize = expoVectorSize;
		// allocate a larger chunk of memory, copy the data and delete the old chunk
		TYPE* const tmp(_vector);
		_vector = (TYPE*) operator new[] (newVectorSize * sizeof(TYPE));
		_ArrayMoveConstruct<true>(_vector, tmp, _size);
		_vectorSize = newVectorSize;
		operator delete[] (tmp);
	}

	// Decrease the size of the array at the specified new size.
	inline void		_Shrink(IDX newSize)
	{
		ASSERT(newSize <= _size);
		_ArrayDestruct(_vector+newSize, _size-newSize);
		_size = newSize;
	}
	inline void		_ShrinkExact(IDX newSize)
	{
		_Shrink(newSize);
		_vectorSize = newSize;
		if (newSize == 0) {
			operator delete[] (_vector);
			_vector = NULL;
		} else {
			TYPE* const tmp(_vector);
			_vector = (TYPE*) operator new[] (_vectorSize * sizeof(TYPE));
			_ArrayMoveConstruct<true>(_vector, tmp, _vectorSize);
			operator delete[] (tmp);
		}
	}

	// Implement construct/destruct for the array elements.
	static inline void	_ArrayConstruct(TYPE* RESTRICT dst, IDX n)
	{
		if (useConstruct) {
			while (n--)
				new(dst+n) TYPE;
		}
	}
	static inline void	_ArrayCopyConstruct(TYPE* RESTRICT dst, const TYPE* RESTRICT src, IDX n)
	{
		if (useConstruct) {
			while (n--)
				new(dst+n) TYPE(src[n]);
		} else {
			memcpy((void*)dst, (const void*)src, n*sizeof(TYPE));
		}
	}
	static inline void	_ArrayDestruct(TYPE* dst, IDX n)
	{
		if (useConstruct) {
			while (n--)
				(dst+n)->~TYPE();
		}
	}
	// Implement copy/move for the array elements.
	static inline void	_ArrayCopyRestrict(TYPE* RESTRICT dst, const TYPE* RESTRICT src, IDX n)
	{
		if (useConstruct) {
			while (n--)
				dst[n] = src[n];
		} else {
			memcpy((void*)dst, (const void*)src, n*sizeof(TYPE));
		}
	}
	static inline void	_ArrayMoveConstructFwd(TYPE* dst, TYPE* src, IDX n)
	{
		ASSERT(dst != src);
		if (useConstruct > 1) {
			for (IDX i=0; i<n; ++i) {
				new(dst+i) TYPE(src[i]);
				(src+i)->~TYPE();
			}
		} else {
			memmove((void*)dst, (const void*)src, n*sizeof(TYPE));
		}
	}
	template <bool bRestrict>
	static inline void	_ArrayMoveConstruct(TYPE* dst, TYPE* src, IDX n)
	{
		ASSERT(dst != src);
		if (useConstruct > 1) {
			while (n--) {
				new(dst+n) TYPE(src[n]);
				(src+n)->~TYPE();
			}
		} else {
			const size_t _size(sizeof(TYPE)*n);
			if (bRestrict)
				memcpy((void*)dst, (const void*)src, _size);
			else
				memmove((void*)dst, (const void*)src, _size);
		}
	}
	template <bool bRestrict>
	static inline void	_ArrayMoveCopy(TYPE* dst, TYPE* src, IDX n)
	{
		ASSERT(dst != src);
		if (useConstruct > 1) {
			while (n--) {
				dst[n] = src[n];
				(src+n)->~TYPE();
			}
		} else {
			const size_t _size(sizeof(TYPE)*n);
			if (useConstruct == 1)
				while (n--)
					(dst+n)->~TYPE();
			if (bRestrict)
				memcpy((void*)dst, (const void*)src, _size);
			else
				memmove((void*)dst, (const void*)src, _size);
		}
	}

protected:
	IDX		_size;
	IDX		_vectorSize;
	TYPE*	_vector;

public:
	static const IDX NO_INDEX;

#if _USE_VECTORINTERFACE != 0
public:
	typedef Type value_type;
	typedef value_type* iterator;
	typedef const value_type* const_iterator;
	typedef value_type& reference;
	typedef const value_type& const_reference;
	typedef std::vector<Type> VectorType;
	inline cList(const VectorType& rList) { CopyOf(&rList[0], rList.size()); }
	inline bool empty() const { return IsEmpty(); }
	inline IDX size() const { return GetSize(); }
	inline IDX capacity() const { return GetCapacity(); }
	inline void clear() { Empty(); }
	inline void insert(const_iterator it, const_reference elem) { InsertAt(it-this->_vector, elem); }
	#ifdef _SUPPORT_CPP11
	template <typename... Args>
	inline reference emplace_back(Args&&... args) { return AddConstruct(std::forward<Args>(args)...); }
	inline void push_back(value_type&& elem) { AddConstruct(elem); }
	#endif
	inline void push_back(const_reference elem) { Insert(elem); }
	inline void pop_back() { RemoveLast(); }
	inline void reserve(IDX newSize) { Reserve(newSize); }
	inline void resize(IDX newSize) { Resize(newSize); }
	inline void erase(const_iterator it) { RemoveAtMove(it-this->_vector); }
	inline const_iterator cdata() const { return GetData(); }
	inline const_iterator cbegin() const { return Begin(); }
	inline const_iterator cend() const { return End(); }
	inline iterator data() const { return GetData(); }
	inline iterator begin() const { return Begin(); }
	inline iterator end() const { return End(); }
	inline ArgType front() const { return First(); }
	inline ArgType back() const { return Last(); }
	inline reference front() { return First(); }
	inline reference back() { return Last(); }
	inline void swap(cList& rList) { Swap(rList); }
#endif

#ifdef _USE_BOOST
protected:
	// implement BOOST serialization
	friend class boost::serialization::access;
	template<class Archive>
	void save(Archive& ar, const unsigned int /*version*/) const {
		ar & _size;
		ar & boost::serialization::make_array(_vector, _size);
	}
	template<class Archive>
	void load(Archive& ar, const unsigned int /*version*/) {
		IDX newSize;
		ar & newSize;
		Resize(newSize);
		ar & boost::serialization::make_array(_vector, _size);
	}
	BOOST_SERIALIZATION_SPLIT_MEMBER()
#endif
};
template <typename TYPE, typename ARG_TYPE, int useConstruct, int grow, typename IDX_TYPE>
const typename cList<TYPE,ARG_TYPE,useConstruct,grow,IDX_TYPE>::IDX
cList<TYPE,ARG_TYPE,useConstruct,grow,IDX_TYPE>::NO_INDEX(DECLARE_NO_INDEX(IDX));
/*----------------------------------------------------------------*/

template <typename IDX_TYPE>
inline bool ValidIDX(const IDX_TYPE& idx) {
	return (idx != DECLARE_NO_INDEX(IDX_TYPE));
}
/*----------------------------------------------------------------*/


// some test functions
// run cListTest(99999);
#if 0
static bool SortIntTest(int a, int b) { return a<b; }
inline bool cListTestIter(unsigned elems) {
	std::vector<int> arrR;
	cList<int, int, 0> arr0;
	cList<int, int, 1> arr1;
	cList<int, int, 2> arr2;
	cList<int, int, 1> arrC;
	for (unsigned i=0; i<elems; ++i) {
		const int e = RAND();
		arrR.push_back(e);
		arr0.InsertSort(e);
		arr1.InsertSort(e);
		arr2.InsertSort(e);
		arrC.Insert(e);
	}
	std::sort(arrR.begin(), arrR.end());
	arrC.Sort(SortIntTest);
	for (size_t i=0; i<arrR.size(); ++i) {
		const int e = arrR[i];
		if (arrC[i] != e ||
			arr0[i] != e ||
			arr1[i] != e ||
			arr2[i] != e)
		{
			ASSERT("there is a problem" == NULL);
			return false;
		}
	}
	for (size_t i=0; i<6; ++i) {
		const unsigned nDel = RAND()%arrR.size();
		arrR.erase(arrR.begin()+nDel);
		arr0.RemoveAtMove(nDel);
		arr1.RemoveAtMove(nDel);
		arr2.RemoveAtMove(nDel);
	}
	if (arrR.size() != arr0.GetSize() ||
		arrR.size() != arr1.GetSize() ||
		arrR.size() != arr2.GetSize())
	{
		ASSERT("there is a problem" == NULL);
		return false;
	}
	for (size_t i=0; i<6; ++i) {
		const unsigned nDel = RAND()%arrR.size();
		const unsigned nCount = 1+RAND()%(arrR.size()/10+1);
		if (nDel + nCount >= arrR.size())
			continue;
		arrR.erase(arrR.begin()+nDel, arrR.begin()+nDel+nCount);
		arr0.RemoveAtMove(nDel, nCount);
		arr1.RemoveAtMove(nDel, nCount);
		arr2.RemoveAtMove(nDel, nCount);
	}
	if (arrR.size() != arr0.GetSize() ||
		arrR.size() != arr1.GetSize() ||
		arrR.size() != arr2.GetSize())
	{
		ASSERT("there is a problem" == NULL);
		return false;
	}
	for (size_t i=0; i<arrR.size(); ++i) {
		const int e = arrR[i];
		if (arr0[i] != e ||
			arr1[i] != e ||
			arr2[i] != e)
		{
			ASSERT("there is a problem" == NULL);
			return false;
		}
	}
	cList<int, int, 1> arrS(1+RAND()%(2*elems));
	for (size_t i=0; i<6; ++i) {
		arrS.Insert(RAND());
	}
	arrS.RemoveLast(RAND()%arrS.GetSize());
	arrS.CopyOf(&arrR[0], arrR.size());
	for (size_t i=0; i<6; ++i) {
		arrS.Insert(RAND());
	}
	arrS.RemoveLast(6);
	for (size_t i=0; i<arrR.size(); ++i) {
		const int e = arrR[i];
		if (arrS[i] != e)
		{
			ASSERT("there is a problem" == NULL);
			return false;
		}
	}
	return true;
}
inline bool cListTest(unsigned iters) {
	srand((unsigned)time(NULL));
	for (unsigned i=0; i<iters; ++i) {
		const unsigned elems = 100+RAND()%1000;
		if (!cListTestIter(elems))
			return false;
	}
	return true;
}
#endif
/*----------------------------------------------------------------*/



/**************************************************************************************
 * Fixed size list template
 **************************************************************************************/

template <typename TYPE, int N>
class cListFixed {
public:
	typedef TYPE Type;
	typedef const TYPE& ArgType;
	typedef unsigned IDX;
	enum {MAX_SIZE = N};

	inline cListFixed() : _size(0) {}
	inline void CopyOf(const TYPE* pData, IDX nSize) {
		memcpy(_vector, pData, nSize);
		_size = nSize;
	}
	inline bool IsEmpty() const {
		return (_size == 0);
	}
	inline IDX GetSize() const {
		return _size;
	}
	inline const TYPE* Begin() const {
		return _vector;
	}
	inline TYPE* Begin() {
		return _vector;
	}
	inline const TYPE* End() const {
		return _vector+_size;
	}
	inline TYPE* End() {
		return _vector+_size;
	}
	inline const TYPE& First() const {
		ASSERT(_size > 0);
		return _vector[0];
	}
	inline TYPE& First() {
		ASSERT(_size > 0);
		return _vector[0];
	}
	inline const TYPE& Last() const {
		ASSERT(_size > 0);
		return _vector[_size-1];
	}
	inline TYPE& Last() {
		ASSERT(_size > 0);
		return _vector[_size-1];
	}
	inline const TYPE& operator[](IDX index) const {
		ASSERT(index < _size);
		return _vector[index];
	}
	inline TYPE& operator[](IDX index) {
		ASSERT(index < _size);
		return _vector[index];
	}
	inline TYPE& AddEmpty() {
		ASSERT(_size < N);
		return _vector[_size++];
	}
	inline void InsertAt(IDX index, ArgType elem) {
		ASSERT(_size < N);
		memmove(_vector+index+1, _vector+index, sizeof(TYPE)*(_size++ - index));
		_vector[index] = elem;
	}
	inline void Insert(ArgType elem) {
		ASSERT(_size < N);
		_vector[_size++] = elem;
	}
	inline void RemoveLast() {
		ASSERT(_size);
		--_size;
	}
	inline void RemoveAt(IDX index) {
		ASSERT(index < _size);
		if (index+1 == _size)
			RemoveLast();
		else
			memcpy(_vector+index, _vector+(--_size), sizeof(TYPE));
	}
	inline void Empty() {
		_size = 0;
	}
	inline void Sort() {
		std::sort(Begin(), End());
	}
	inline IDX Find(ArgType elem) const
	{
		IDX i = _size;
		while (i)
			if (_vector[--i] == elem)
				return i;
		return NO_INDEX;
	}
	inline IDX FindFirstEqlGreater(ArgType searchedKey) const {
		if (_size == 0) return 0;
		IDX l1 = 0, l2 = _size;
		do {
			IDX i = (l1 + l2) >> 1;
			const TYPE& key = _vector[i];
			if (key == searchedKey) {
				while (i-- && _vector[i] == searchedKey);
				return i+1;
			}
			if (key < searchedKey)
				l1 = i+1;
			else
				l2 = i;
		} while (l1 < l2);
		return l1;
	}
	inline void StoreTop(ArgType elem) {
		const IDX idx(FindFirstEqlGreater(elem));
		if (idx < GetSize()) {
			if (GetSize() >= N)
				RemoveLast();
			InsertAt(idx, elem);
		} else if (GetSize() < N) {
			Insert(elem);
		}
	}

protected:
	TYPE _vector[N];
	IDX _size;

public:
	static const IDX NO_INDEX;

#ifdef _USE_BOOST
protected:
	// implement BOOST serialization
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & _size;
		ar & boost::serialization::make_array(_vector, _size);
	}
#endif
};
template <typename TYPE, int N>
const typename cListFixed<TYPE,N>::IDX cListFixed<TYPE,N>::NO_INDEX(DECLARE_NO_INDEX(IDX));
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_LIST_H__
