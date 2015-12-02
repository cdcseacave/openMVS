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
#define FOREACH(var, arr) for (ARR2IDX(arr) var=0, var##size=(arr).GetSize(); var<var##size; ++var)
#endif
#ifndef RFOREACH
#define RFOREACH(var, arr) for (ARR2IDX(arr) var=(arr).GetSize(); var-->0; )
#endif
// cList iterator by pointer
#ifndef FOREACHPTR
#define FOREACHPTR(var, arr) for (auto var=(arr).Begin(), var##end=(arr).End(); var!=var##end; ++var)
#endif
#ifndef RFOREACHPTR
#define RFOREACHPTR(var, arr) for (auto var=(arr).End(), var##begin=(arr).Begin(); var--!=var##begin; )
#endif

// raw data array iterator by index
#ifndef FOREACHRAW
#define FOREACHRAW(var, sz) for (IDX var=0, var##size=(sz); var<var##size; ++var)
#endif
#ifndef RFOREACHRAW
#define RFOREACHRAW(var, sz) for (IDX var=sz; var-->0; )
#endif
// raw data array iterator by pointer
#ifndef FOREACHRAWPTR
#define FOREACHRAWPTR(var, arr, sz) for (auto var=(arr), var##end=var+sz; var!=var##end; ++var)
#endif
#ifndef RFOREACHRAWPTR
#define RFOREACHRAWPTR(var, arr, sz) for (auto var##begin=(arr), var=var##begin+sz; var--!=var##begin; )
#endif

// constructs a cList reference to a given raw data array
#ifndef CLISTREFRAW
#define CLISTREFRAW(CLIST, var, arr, sz) uint8_t _ArrData##var[sizeof(CLIST)]; new(_ArrData##var) CLIST(sz, arr); const CLIST& var(*((const CLIST*)_ArrData##var))
#endif
// constructs a cList reference to a given std::vector
#ifndef CLISTREFVECTOR
#define CLISTREFVECTOR(CLIST, var, vec) uint8_t _ArrData##var[sizeof(CLIST)]; new(_ArrData##var) CLIST(vec.size(), &vec[0]); const CLIST& var(*((const CLIST*)_ArrData##var))
#endif

#define CLISTDEF0(TYPE) SEACAVE::cList< TYPE, const TYPE&, 0 >
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
	inline cList() : size(0), vectorSize(0), vector(NULL)
	{
	}

	// construct a list containing _size initialized elements
	cList(IDX _size) : size(_size), vectorSize(_size), vector((TYPE*)operator new[] (_size * sizeof(TYPE)))
	{
		ASSERT(_size > 0 && _size < NO_INDEX);
		_ArrayConstruct(vector, _size);
	}

	// construct a list containing _size initialized elements and allocated space for _reserved elements
	cList(IDX _size, IDX _reserved) : size(_size), vectorSize(_reserved), vector((TYPE*)operator new[] (_reserved * sizeof(TYPE)))
	{
		ASSERT(_reserved >= _size && _reserved < NO_INDEX);
		_ArrayConstruct(vector, _size);
	}

	// copy constructor: creates a deep-copy of the given list
	cList(const cList& rList) : size(rList.size), vectorSize(rList.vectorSize), vector(NULL)
	{
		if (vectorSize == 0) {
			ASSERT(size == 0);
			return;
		}
		vector = (TYPE*)(operator new[] (vectorSize * sizeof(TYPE)));
		_ArrayCopyConstruct(vector, rList.vector, size);
	}

	// constructor a list from a raw data array, taking ownership of the array memory
	explicit inline cList(IDX nSize, TYPE* pData) : size(nSize), vectorSize(nSize), vector(pData)
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
		if (bForceResize || vectorSize < rList.vectorSize) {
			_Release();
			vectorSize = rList.vectorSize;
			vector = (TYPE*) operator new[] (vectorSize * sizeof(TYPE));
			_ArrayCopyConstruct(vector, rList.vector, rList.size);
		} else {
			if (size >= rList.size) {
				_ArrayDestruct(vector+rList.size, size-rList.size);
				_ArrayCopyRestrict(vector, rList.vector, rList.size);
			} else {
				_ArrayCopyRestrict(vector, rList.vector, size);
				_ArrayCopyConstruct(vector+size, rList.vector+size, rList.size-size);
			}
		}
		size = rList.size;
		return (*this);
	}

	inline	cList&	CopyOf(const TYPE* pData, IDX nSize, bool bForceResize=false)
	{
		if (vector == pData)
			return (*this);
		if (bForceResize || vectorSize < nSize) {
			_Release();
			vectorSize = nSize;
			vector = (TYPE*) operator new[] (vectorSize * sizeof(TYPE));
			_ArrayCopyConstruct(vector, pData, nSize);
		} else {
			if (size >= nSize) {
				_ArrayDestruct(vector+nSize, size-nSize);
				_ArrayCopyRestrict(vector, pData, nSize);
			} else {
				_ArrayCopyRestrict(vector, pData, size);
				_ArrayCopyConstruct(vector+size, pData+size, nSize-size);
			}
		}
		size = nSize;
		return (*this);
	}

	// release current list and swap the content with the given list
	inline	cList&	CopyOfRemove(cList& rList)
	{
		if (this == &rList)
			return (*this);
		_Release();
		size = rList.size;
		vectorSize = rList.vectorSize;
		vector = rList.vector;
		rList.vector = NULL;
		rList.size = rList.vectorSize = 0;
		return (*this);
	}

	inline	void	Join(const cList& rList)
	{
		if (this == &rList || rList.size == 0)
			return;
		const IDX newSize = size + rList.size;
		Reserve(newSize);
		_ArrayCopyConstruct(vector+size, rList.vector, rList.size);
		size = newSize;
	}
	inline	void	Join(const TYPE* pData, IDX nSize)
	{
		const IDX newSize = size + nSize;
		Reserve(newSize);
		_ArrayCopyConstruct(vector+size, pData, nSize);
		size = newSize;
	}

	inline	void	JoinRemove(cList& rList)
	{
		if (this == &rList || rList.size == 0)
			return;
		const IDX newSize(size + rList.size);
		Reserve(newSize);
		_ArrayMoveConstruct<true>(vector+size, rList.vector, rList.size);
		size = newSize;
		rList.size = 0;
	}

	// Swap the elements of the two lists.
	inline	void	Swap(cList& rList)
	{
		if (this == &rList)
			return;
		const IDX tmpSize = size;
		size = rList.size;
		rList.size = tmpSize;
		const IDX tmpVectorSize = vectorSize;
		vectorSize = rList.vectorSize;
		rList.vectorSize = tmpVectorSize;
		TYPE* const tmpVector = vector;
		vector = rList.vector;
		rList.vector = tmpVector;
	}

	// Swap the two elements.
	inline	void	Swap(IDX idx1, IDX idx2)
	{
		ASSERT(idx1 < size && idx2 < size);
		TYPE tmp = vector[idx1];
		vector[idx1] = vector[idx2];
		vector[idx2] = tmp;
	}

	// Set the allocated memory (normally used for types without constructor).
	inline void		Memset(uint8_t val)
	{
		memset(vector, val, size * sizeof(TYPE));
	}
	inline void		MemsetValue(ARG_TYPE val)
	{
		std::fill_n(vector, size, val);
	}

	// Delete the old array, and create a new one of the desired size;
	// the old elements are deleted and the array is filled with new empty ones.
	inline void		Reset(IDX newSize)
	{
		_Release();
		vectorSize = newSize;
		size = newSize;
		if (newSize == 0) {
			vector = NULL;
			return;
		}
		vector = (TYPE*) operator new[] (newSize * sizeof(TYPE));
		_ArrayConstruct(vector, newSize);
	}

	// Pre-allocate memory for the array at the desired size;
	// the old elements are kept.
	inline	void	Reserve(IDX needVectorSize)
	{
		if (vectorSize < needVectorSize)
			_Grow(needVectorSize);
	}
	// Same as above, but only the extra needed space is passed.
	inline	void	ReserveExtra(IDX needVectorExtraSize)
	{
		if (vectorSize < size+needVectorExtraSize)
			_Grow(vectorSize+MAXF(needVectorExtraSize,(IDX)grow));
	}

	// Set the size of the array at the new desired size;
	// the old elements are kept, but only the ones that are below the new size.
	// If increased, the array is filled with new empty elements.
	inline	void	Resize(IDX newSize)
	{
		if (newSize == size)
			return;
		if (newSize < size)
			return _Shrink(newSize);
		Reserve(newSize);
		_ArrayConstruct(vector+size, newSize-size);
		size = newSize;
	}
	inline	void	ResizeExact(IDX newSize)
	{
		if (newSize == size)
			return;
		if (newSize < size)
			return _ShrinkExact(newSize);
		Reserve(newSize);
		_ArrayConstruct(vector+size, newSize-size);
		size = newSize;
	}

	// Free unused memory.
	inline void		ReleaseFree()
	{
		if (size < vectorSize)
			_ShrinkExact(size);
	}

	inline IDX		GetIndex(const TYPE* pElem) const
	{
		ASSERT(pElem-vector < size);
		return (IDX)(pElem-vector);
	}

	inline const TYPE&	operator[](IDX index) const
	{
		ASSERT(index < size);
		return vector[index];
	}
	inline TYPE&	operator[](IDX index)
	{
		ASSERT(index < size);
		return vector[index];
	}

	inline TYPE*	GetData() const
	{
		return vector;
	}
	inline size_t	GetDataSize() const
	{
		return sizeof(TYPE)*size;
	}

	inline TYPE*	Begin() const
	{
		return vector;
	}

	inline TYPE*	End() const
	{
		return vector+size;
	}

	inline const TYPE&	First() const
	{
		ASSERT(size > 0);
		return vector[0];
	}
	inline TYPE&	First()
	{
		ASSERT(size > 0);
		return vector[0];
	}

	inline const TYPE&	Last() const
	{
		ASSERT(size > 0);
		return vector[size-1];
	}
	inline TYPE&	Last()
	{
		ASSERT(size > 0);
		return vector[size-1];
	}

	// Adds a new empty element at the end of the array.
	inline TYPE&	AddEmpty()
	{
		if (vectorSize <= size)
			_Grow(vectorSize + grow);
		if (useConstruct)
			return *(new(vector + (size++)) TYPE);
		else
			return vector[size++];
	}
	inline TYPE*	AddEmpty(IDX numElems)
	{
		const IDX newSize = size + numElems;
		if (vectorSize < newSize)
			_Grow(newSize + grow);
		Type* const pData = vector + size;
		_ArrayConstruct(pData, numElems);
		size = newSize;
		return pData;
	}

	#ifdef _SUPPORT_CPP11
	// Adds a new empty element at the end of the array and pass the arguments to its constructor.
	template <typename... Args>
	inline TYPE&	AddConstruct(Args&&... args)
	{
		if (vectorSize <= size)
			_Grow(vectorSize + grow);
		return *(new(vector + (size++)) TYPE(std::forward<Args>(args)...));
	}
	#endif

	inline IDX		InsertEmpty()
	{
		if (vectorSize <= size)
			_Grow(vectorSize + grow);
		if (useConstruct)
			new(vector + size) TYPE;
		return size++;
	}

	// Adds the new element at the end of the array.
	#ifdef _SUPPORT_CPP11
	template <typename T>
	inline void		Insert(T&& elem)
	{
		if (vectorSize <= size)
			_Grow(vectorSize + grow);
		if (useConstruct)
			new(vector+(size++)) TYPE(std::forward<T>(elem));
		else
			vector[size++] = std::forward<T>(elem);
	}
	#else
	inline void		Insert(ARG_TYPE elem)
	{
		if (vectorSize <= size)
			_Grow(vectorSize + grow);
		if (useConstruct)
			new(vector+(size++)) TYPE(elem);
		else
			vector[size++] = elem;
	}
	#endif

	#ifdef _SUPPORT_CPP11
	template <typename T>
	inline void		SetAt(IDX index, T&& elem)
	{
		if (size <= index)
			Resize(index + 1);
		vector[index] = std::forward<T>(elem);
	}
	#else
	inline void		SetAt(IDX index, ARG_TYPE elem)
	{
		if (size <= index)
			Resize(index + 1);
		vector[index] = elem;
	}
	#endif

	#ifdef _SUPPORT_CPP11
	template <typename T>
	inline void		AddAt(IDX index, T&& elem)
	{
		if (index < size)
			return InsertAt(index, std::forward<T>(elem));
		const IDX newSize = index + 1;
		if (vectorSize <= newSize)
			_Grow(newSize + grow);
		_ArrayConstruct(vector+size, index-size);
		if (useConstruct)
			new(vector+index) TYPE(std::forward<T>(elem));
		else
			vector[index] = std::forward<T>(elem);
		++size;
	}
	#else
	inline void		AddAt(IDX index, ARG_TYPE elem)
	{
		if (index < size)
			return InsertAt(index, elem);
		const IDX newSize = index + 1;
		if (vectorSize <= newSize)
			_Grow(newSize + grow);
		_ArrayConstruct(vector+size, index-size);
		if (useConstruct)
			new(vector+index) TYPE(elem);
		else
			vector[index] = elem;
		++size;
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
		if (vectorSize <= size)
			_Grow(vectorSize + grow);
		return vector+size++;
	}
	inline TYPE*	AllocateAt(IDX index)
	{
		ASSERT(index <= size);
		const IDX move(size-index);
		Allocate();
		_ArrayMoveConstruct<false>(vector+index+1, vector+index, move);
		return vector+index;
	}

	inline IDX		InsertSort(ARG_TYPE elem)
	{
		IDX l1(0), l2(size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE compElem(vector[i]);
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
		IDX l1(0), l2(size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE compElem(vector[i]);
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
		IDX l1(0), l2(size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			const int res(xCompare(vector+i, &elem));
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
		return GetNth(size >> 1);
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
		if (size < 2)
			return true;
		IDX i = size-1;
		do {
			ARG_TYPE elem1 = vector[i];
			ARG_TYPE elem0 = vector[--i];
			if (!(elem0 < elem1 || elem0 == elem1))
				return false;
		} while (i > 0);
		return true;
	}
	inline	bool	IsSorted(TFncCompareBool xCompare) const
	{
		if (size < 2)
			return true;
		IDX i = size-1;
		do {
			ARG_TYPE elem1 = vector[i];
			ARG_TYPE elem0 = vector[--i];
			if (!xCompare(elem0, elem1))
				return false;
		} while (i > 0);
		return true;
	}

	inline std::pair<IDX,bool>	InsertSortUnique(ARG_TYPE elem)
	{
		IDX l1(0), l2(size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE compElem(vector[i]);
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
		IDX l1(0), l2(size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE compElem(vector[i]);
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
		IDX l1(0), l2(size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			const int res(xCompare(vector+i, &elem));
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
		if (idx < size) {
			if (size >= N)
				RemoveLast();
			InsertAt(idx, elem);
		} else if (size < N) {
			Insert(elem);
		}
	}
	inline void		StoreTop(ARG_TYPE elem, IDX N) {
		const IDX idx(FindFirstEqlGreater(elem));
		if (idx < size) {
			if (size >= N)
				RemoveLast();
			InsertAt(idx, elem);
		} else if (size < N) {
			Insert(elem);
		}
	}

	inline IDX		FindFirst(ARG_TYPE searchedKey) const
	{
		IDX l1(0), l2(size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(vector[i]);
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
		IDX l1(0), l2(size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(vector[i]);
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
		IDX l1(0), l2(size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(vector[i]);
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
		IDX l1(0), l2(size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(vector[i]);
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
		IDX l1(0), l2(size);
		while (l1 < l2) {
			IDX i((l1 + l2) >> 1);
			const int res(xCompare(vector+i, searchedKey));
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
		if (size == 0) return NO_INDEX;
		IDX l1(0), l2(size);
		do {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(vector[i]);
			if (searchedKey < key)
				l2 = i;
			else if (key < searchedKey)
				l1 = i+1;
			else {
				while (i-- && !(vector[i] < searchedKey));
				return i;
			}
		} while (l1 < l2);
		return l1-1;
	}

	template <typename SEARCH_TYPE>
	inline IDX		FindFirstBelow(const SEARCH_TYPE& searchedKey) const
	{
		if (size == 0) return NO_INDEX;
		IDX l1(0), l2(size);
		do {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(vector[i]);
			if (key == searchedKey) {
				while (i-- && vector[i] == searchedKey);
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
		if (size == 0) return 0;
		IDX l1(0), l2(size);
		do {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(vector[i]);
			if (searchedKey < key)
				l2 = i;
			else if (key < searchedKey)
				l1 = i+1;
			else {
				while (i-- && !(vector[i] < searchedKey));
				return i+1;
			}
		} while (l1 < l2);
		return l1;
	}

	template <typename SEARCH_TYPE>
	inline IDX		FindFirstEqlGreater(const SEARCH_TYPE& searchedKey) const
	{
		if (size == 0) return 0;
		IDX l1(0), l2(size);
		do {
			IDX i((l1 + l2) >> 1);
			ARG_TYPE key(vector[i]);
			if (key == searchedKey) {
				while (i-- && vector[i] == searchedKey);
				return i+1;
			}
			if (key < searchedKey)
				l1 = i+1;
			else
				l2 = i;
		} while (l1 < l2);
		return l1;
	}

	// Find the matching "elem" by brute-force.
	inline IDX		Find(ARG_TYPE elem) const
	{
		IDX i = size;
		while (i)
			if (vector[--i] == elem)
				return i;
		return NO_INDEX;
	}

	template <typename SEARCH_TYPE>
	inline IDX		Find(const SEARCH_TYPE& searchedKey) const
	{
		IDX i = size;
		while (i)
			if (vector[--i] == searchedKey)
				return i;
		return NO_INDEX;
	}

	inline	IDX		Find(const void* searchedKey, TFncCompare xCompare) const
	{
		for (IDX i = 0; i < size; i++)
			if (xCompare(vector + i, searchedKey) == 0)
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
		IDX i = size;
		while (i)
			if (vector[--i] == elem)
				RemoveAt(i);
	}

	inline ARG_TYPE	RemoveTail()
	{
		ASSERT(size);
		ASSERT(!useConstruct);
		return vector[--size];
	}

	inline void		RemoveLast()
	{
		ASSERT(size);
		_ArrayDestruct(vector+(--size), 1);
	}

	inline void		RemoveLast(IDX count)
	{
		ASSERT(count <= size);
		_ArrayDestruct(vector+(size-=count), count);
	}

	inline void		RemoveAt(IDX index)
	{
		ASSERT(index < size);
		if (index+1 == size)
			RemoveLast();
		else
			_ArrayMoveCopy<true>(vector+index, vector+(--size), 1);
	}

	inline void		RemoveAt(IDX index, IDX count)
	{
		ASSERT(index+count <= size);
		if (index+count == size) {
			RemoveLast(count);
		} else {
			size -= count;
			const IDX move(size-index);
			TYPE* const vectorEnd(vector+size);
			if (move < count) {
				const IDX del(count-move);
				_ArrayMoveCopy<true>(vector+index, vectorEnd+del, move);
				_ArrayDestruct(vectorEnd, del);
			} else {
				_ArrayMoveCopy<false>(vector+index, vectorEnd, count);
			}
		}
	}

	inline void		RemoveAtMove(IDX index)
	{
		ASSERT(index < size);
		if (index+1 == size) {
			RemoveLast();
		} else {
			_ArrayDestruct(vector+index, 1);
			_ArrayMoveConstructFwd(vector+index, vector+index+1, (--size)-index);
		}
	}

	inline void		RemoveAtMove(IDX index, IDX count)
	{
		ASSERT(index+count <= size);
		if (index+count == size) {
			RemoveLast(count);
		} else {
			_ArrayDestruct(vector+index, count);
			_ArrayMoveConstructFwd(vector+index, vector+index+count, (size-=count)-index);
		}
	}

	inline	void	Empty()
	{
		_ArrayDestruct(vector, size);
		size = 0;
	}

	// same as Empty(), plus free all allocated memory
	inline	void	Release()
	{
		_Release();
		_Init();
	}

	// Discard all stored data and initialize it as an empty array.
	// (note: call this only when you know what you are doing,
	// you have to deallocate yourself all elements and vector data)
	inline void		Reset()
	{
		_Init();
	}

	// Delete also the pointers (take care to use this function only if the elements are pointers).
	inline void		EmptyDelete()
	{
		while (size)
			delete vector[--size];
	}

	// same as EmptyDelete(), plus free all allocated memory
	inline void		ReleaseDelete()
	{
		EmptyDelete();
		operator delete[] (vector);
		vector = NULL;
		vectorSize = 0;
	}

	inline	IDX		GetCapacity() const
	{
		return vectorSize;
	}

	inline	IDX		GetSize() const
	{
		return size;
	}

	inline	bool	IsEmpty() const
	{
		return (size == 0);
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
		_ArrayDestruct(vector, size);
		operator delete[] (vector);
	}

	// Initialize array.
	inline	void	_Init()
	{
		vector = NULL;
		vectorSize = size = 0;
	}

	// Increase the size of the array at least to the specified amount.
	inline void		_Grow(IDX newVectorSize)
	{
		ASSERT(newVectorSize > vectorSize);
		// grow by 50% or at least to minNewVectorSize
		const IDX expoVectorSize(vectorSize + (vectorSize>>1));
		if (newVectorSize < expoVectorSize)
			newVectorSize = expoVectorSize;
		// allocate a larger chunk of memory, copy the data and delete the old chunk
		TYPE* const tmp(vector);
		vector = (TYPE*) operator new[] (newVectorSize * sizeof(TYPE));
		_ArrayMoveConstruct<true>(vector, tmp, size);
		vectorSize = newVectorSize;
		operator delete[] (tmp);
	}

	// Decrease the size of the array at the specified new size.
	inline void		_Shrink(IDX newSize)
	{
		ASSERT(newSize <= size);
		_ArrayDestruct(vector+newSize, size-newSize);
		size = newSize;
	}
	inline void		_ShrinkExact(IDX newSize)
	{
		_Shrink(newSize);
		vectorSize = newSize;
		if (newSize == 0) {
			operator delete[] (vector);
			vector = NULL;
		} else {
			TYPE* const tmp(vector);
			vector = (TYPE*) operator new[] (vectorSize * sizeof(TYPE));
			_ArrayMoveConstruct<true>(vector, tmp, vectorSize);
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
			memcpy(dst, src, n*sizeof(TYPE));
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
			memcpy(dst, src, n*sizeof(TYPE));
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
			memmove(dst, src, n*sizeof(TYPE));
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
			const size_t size(sizeof(TYPE)*n);
			if (bRestrict)
				memcpy(dst, src, size);
			else
				memmove(dst, src, size);
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
			const size_t size(sizeof(TYPE)*n);
			if (useConstruct == 1)
				while (n--)
					(dst+n)->~TYPE();
			if (bRestrict)
				memcpy(dst, src, size);
			else
				memmove(dst, src, size);
		}
	}

protected:
	IDX		size;
	IDX		vectorSize;
	TYPE*	vector;

public:
	static const IDX NO_INDEX;

#ifdef _USE_BOOST
protected:
	// implement BOOST serialization
	friend class boost::serialization::access;
	template<class Archive>
	void save(Archive& ar, const unsigned int /*version*/) const
	{
		ar & size;
		ar & boost::serialization::make_array(vector, size);
	}
	template<class Archive>
	void load(Archive& ar, const unsigned int /*version*/)
	{
		IDX newSize;
		ar & newSize;
		Resize(newSize);
		ar & boost::serialization::make_array(vector, size);
	}
	BOOST_SERIALIZATION_SPLIT_MEMBER()
#endif
};
template <typename TYPE, typename ARG_TYPE, int useConstruct, int grow, typename IDX_TYPE>
const typename cList<TYPE,ARG_TYPE,useConstruct,grow,IDX_TYPE>::IDX
cList<TYPE,ARG_TYPE,useConstruct,grow,IDX_TYPE>::NO_INDEX(DECLARE_NO_INDEX(IDX));
/*----------------------------------------------------------------*/


template<class VECTOR>
class cList2vector: public VECTOR {
public:
	typedef VECTOR Vector;
	typedef typename VECTOR::IDX IDX;
	typedef typename VECTOR::Type Type;
	typedef typename VECTOR::ArgType ArgType;
	typedef Type value_type;
	typedef value_type* iterator;
	typedef const value_type* const_iterator;
	typedef std::vector<Type> VectorType;
	inline cList2vector() {}
	inline cList2vector(IDX _size) : VECTOR(_size) {}
	inline cList2vector(IDX _size, ArgType _val) : VECTOR(_size) { VECTOR::MemsetValue(_val); }
	inline cList2vector(const VECTOR& rList) : VECTOR(rList) {}
	inline cList2vector(const VectorType& rList) { VECTOR::CopyOf(&rList[0], rList.size()); }
	inline bool empty() const { return VECTOR::IsEmpty(); }
	inline size_t size() const { return VECTOR::GetSize(); }
	inline size_t capacity() const { return VECTOR::GetCapacity(); }
	inline void clear() { VECTOR::Empty(); }
	inline void insert(Type* it, ArgType elem) { VECTOR::InsertAt(it-this->vector, elem); }
	inline void push_back(ArgType elem) { VECTOR::Insert(elem); }
	inline void pop_back() { VECTOR::RemoveLast(); }
	inline void reserve(size_t newSize) { VECTOR::Reserve(newSize); }
	inline void resize(size_t newSize) { VECTOR::Resize(newSize); }
	inline void erase(Type* it) { VECTOR::RemoveAtMove(it-this->vector); }
	inline const Type* cdata() const { return VECTOR::GetData(); }
	inline const Type* cbegin() const { return VECTOR::Begin(); }
	inline const Type* cend() const { return VECTOR::End(); }
	inline Type* data() const { return VECTOR::GetData(); }
	inline Type* begin() const { return VECTOR::Begin(); }
	inline Type* end() const { return VECTOR::End(); }
	inline ArgType front() const { return VECTOR::First(); }
	inline ArgType back() const { return VECTOR::Last(); }
	inline void swap(VECTOR& list) { VECTOR::Swap(list); }
};
#define VECTORINTERFACE(CLIST, var) ((const cList2vector<CLIST>&)(var))
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

	inline cListFixed() : size(0) {}
	inline void CopyOf(const TYPE* pData, IDX nSize) {
		memcpy(vector, pData, nSize);
		size = nSize;
	}
	inline bool IsEmpty() const {
		return (size == 0);
	}
	inline IDX GetSize() const {
		return size;
	}
	inline const TYPE* Begin() const {
		return vector;
	}
	inline TYPE* Begin() {
		return vector;
	}
	inline const TYPE* End() const {
		return vector+size;
	}
	inline TYPE* End() {
		return vector+size;
	}
	inline const TYPE& First() const {
		ASSERT(size > 0);
		return vector[0];
	}
	inline TYPE& First() {
		ASSERT(size > 0);
		return vector[0];
	}
	inline const TYPE& Last() const {
		ASSERT(size > 0);
		return vector[size-1];
	}
	inline TYPE& Last() {
		ASSERT(size > 0);
		return vector[size-1];
	}
	inline const TYPE& operator[](IDX index) const {
		ASSERT(index < size);
		return vector[index];
	}
	inline TYPE& operator[](IDX index) {
		ASSERT(index < size);
		return vector[index];
	}
	inline TYPE& AddEmpty() {
		ASSERT(size < N);
		return vector[size++];
	}
	inline void InsertAt(IDX index, ArgType elem) {
		ASSERT(size < N);
		memmove(vector+index+1, vector+index, sizeof(TYPE)*(size++ - index));
		vector[index] = elem;
	}
	inline void Insert(ArgType elem) {
		ASSERT(size < N);
		vector[size++] = elem;
	}
	inline void RemoveLast() {
		ASSERT(size);
		--size;
	}
	inline void RemoveAt(IDX index) {
		ASSERT(index < size);
		if (index+1 == size)
			RemoveLast();
		else
			memcpy(vector+index, vector+(--size), sizeof(TYPE));
	}
	inline void Empty() {
		size = 0;
	}
	inline void Sort() {
		std::sort(Begin(), End());
	}
	inline IDX Find(ArgType elem) const
	{
		IDX i = size;
		while (i)
			if (vector[--i] == elem)
				return i;
		return NO_INDEX;
	}
	inline IDX FindFirstEqlGreater(ArgType searchedKey) const {
		if (size == 0) return 0;
		IDX l1 = 0, l2 = size;
		do {
			IDX i = (l1 + l2) >> 1;
			const TYPE& key = vector[i];
			if (key == searchedKey) {
				while (i-- && vector[i] == searchedKey);
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
	TYPE vector[N];
	IDX size;

public:
	static const IDX NO_INDEX;

#ifdef _USE_BOOST
protected:
	// implement BOOST serialization
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & size;
		ar & boost::serialization::make_array(vector, size);
	}
#endif
};
template <typename TYPE, int N>
const typename cListFixed<TYPE,N>::IDX cListFixed<TYPE,N>::NO_INDEX(DECLARE_NO_INDEX(IDX));
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_LIST_H__
