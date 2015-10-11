////////////////////////////////////////////////////////////////////
// SharedPtr.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_SHAREDPTR_H__
#define __SEACAVE_SHAREDPTR_H__


// I N C L U D E S /////////////////////////////////////////////////



// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

/**************************************************************************************
 * CSharedPtr template
 * ---------------
 * shared smart pointer
 **************************************************************************************/

template<class TYPE>
struct SharedRef {
	typedef TYPE		TYPE_REF;

	#ifdef SEACAVE_NO_MULTITHREAD
	TYPE_REF			val;
	#else
	volatile TYPE_REF	val;
	#endif

	inline SharedRef() {}
	inline SharedRef(TYPE_REF v) : val(v) {}

	inline TYPE_REF Inc() {
		#ifdef SEACAVE_NO_MULTITHREAD
		ASSERT(val >= 0);
		return ++val;
		#else
		return Thread::safeInc(val);
		#endif
	}
	inline TYPE_REF Dec() {
		#ifdef SEACAVE_NO_MULTITHREAD
		ASSERT(val > 0);
		return --val;
		#else
		return Thread::safeDec(val);
		#endif
	}

	#ifdef _USE_BOOST
	// serialize
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		#ifdef SEACAVE_NO_MULTITHREAD
		ar & val;
		#else
		ar & (TYPE_REF&)val;
		#endif
	}
	#endif
};
/*----------------------------------------------------------------*/


template<class TYPE>
class CSharedPtr
{
protected:
	typedef SharedRef<int32_t>	TYPE_REF;
	typedef TYPE*				TYPE_PTR;

public:
	inline CSharedPtr() : m_pointer(NULL), m_pNoRef(NULL)
	{	// empty construct
	}

	inline explicit CSharedPtr(TYPE_PTR _Ptr) : m_pointer(_Ptr), m_pNoRef(_Ptr ? new TYPE_REF(1) : NULL)
	{	// construct from object pointer
	}

	inline CSharedPtr(const CSharedPtr& _Right) : m_pointer(_Right.m_pointer), m_pNoRef(_Right.m_pNoRef)
	{	// construct by assuming pointer from _Right CSharedPtr
		IncRef();
	}

	inline ~CSharedPtr()
	{	// destroy the object
		DecRef();
	}

	CSharedPtr& operator=(const CSharedPtr& _Right)
	{	// assign compatible _Right (assume pointer)
		if (this != &_Right)
		{
			ASSERT(m_pointer != _Right.m_pointer || m_pNoRef == _Right.m_pNoRef);
			DecRef();
			m_pointer = _Right.m_pointer;
			m_pNoRef = _Right.m_pNoRef;
			IncRef();
		}
		return (*this);
	}

	CSharedPtr& operator=(TYPE_PTR _Ptr)
	{	// assign compatible _Right (assume pointer)
		if (m_pointer != _Ptr)
		{
			DecRef();
			m_pointer = _Ptr;
			m_pNoRef = (_Ptr ? new TYPE_REF(1) : NULL);
		}
		return (*this);
	}

	inline TYPE& operator*() const
	{	// return designated value
		ASSERT(m_pointer);
		return (*m_pointer);
	}

	inline TYPE* operator->() const
	{	// return pointer to class object
		ASSERT(m_pointer);
		return m_pointer;
	}

	inline operator TYPE_PTR() const
	{	// return pointer to class object
		return m_pointer;
	}

	inline bool operator==(const CSharedPtr& _Right) const
	{	// return pointer to class object
		return (m_pointer == _Right.m_pointer);
	}

	inline bool operator!=(const CSharedPtr& _Right) const
	{	// return pointer to class object
		return (m_pointer != _Right.m_pointer);
	}

	inline bool operator==(const void* _Right) const
	{	// return pointer to class object
		return (m_pointer == _Right);
	}

	inline bool operator!=(const void* _Right) const
	{	// return pointer to class object
		return (m_pointer != _Right);
	}

	void Release()
	{	// release pointer
		DecRef();
		m_pointer = NULL;
		m_pNoRef = NULL;
	}

protected:
	inline void IncRef()
	{
		if (m_pointer == NULL)
			return;
		ASSERT(m_pNoRef);
		m_pNoRef->Inc();
	}

	inline void DecRef()
	{
		if (m_pointer == NULL)
			return;
		ASSERT(m_pNoRef);
		if (m_pNoRef->Dec() == 0)
		{
			delete m_pointer;
			m_pointer = NULL;
			delete m_pNoRef;
			m_pNoRef = NULL;
		}
	}

	TYPE_PTR	m_pointer;		// the wrapped object pointer
	TYPE_REF*	m_pNoRef;		// number of references to this pointer

#ifdef _USE_BOOST
protected:
	// implement BOOST serialization
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & m_pointer;
		ar & m_pNoRef;
	}
#endif
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_SHAREDPTR_H__
