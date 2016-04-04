////////////////////////////////////////////////////////////////////
// SML.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_SML_H__
#define __SEACAVE_SML_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "Filters.h"


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

typedef struct SMLVALUE_TYPE {
	String val; //item's value
	void* data; //user data
	SMLVALUE_TYPE() : data(NULL) {}
	SMLVALUE_TYPE(const SMLVALUE_TYPE& r) : data(r.data) { ((SMLVALUE_TYPE&)r).data = NULL; }
	~SMLVALUE_TYPE() { delete data; }
} SMLVALUE;

class SML;
typedef SML* LPSML;
typedef cList<LPSML, LPSML, 0, 8> LPSMLARR;

//typedef cHashTable<SMLVALUE> SMLITEMMAP;
typedef cMapWrap<String, SMLVALUE> SMLITEMMAP;

/**************************************************************************************
 * Simple Markup Language
 * --------------
 * fast and easy to use markup language;
 * contains a map of (name, value) pairs
 **************************************************************************************/

class GENERAL_API SML : public SMLITEMMAP
{
public:
	typedef void (STCALL *TFncInitItem)(const String&, SMLVALUE&, void*);
	typedef bool (STCALL *TFncSaveItem)(const SMLVALUE&, void*);
	typedef void (STCALL *TFncReleaseItem)(SMLVALUE&, void*);

	enum SAVEFLAG {
		NONE		= 0,
		SAVEEMPTY	= (1 << 0), //save empty children
		SORT		= (1 << 1), //sort all entries alphabetically
		SORTINV		= (1 << 2), //sort all entries inverse alphabetically
	};

public:
	SML(const String& =String());
	~SML();

	void			Release();

	// main methods
	bool			Load(const String&);
	bool			Load(ISTREAM&);
	bool			Save(const String&, SAVEFLAG=NONE) const;
	bool			Save(OSTREAM&, SAVEFLAG=NONE) const;
	IDX				CreateChild(const String& =String());
	IDX				CreateChildUnique(const String& =String());
	IDX				InsertChild(const LPSML);
	IDX				InsertChildUnique(const LPSML);
	void			RemoveChild(IDX);
	inline void		RemoveChild(const String& name)					{ RemoveChild(GetChild(name)); }
	void			DestroyChild(IDX);
	inline void		DestroyChild(const String& name)				{ DestroyChild(GetChild(name)); }
	IDX				GetChild(const String&) const;
	const SMLVALUE*	GetValue(const String&) const;
	SMLVALUE&		GetValue(const String&);
	inline SMLVALUE& operator[] (const String& key)					{ return GetValue(key); }

	// misc methods
	inline const String&	GetName() const							{ return m_strName; }
	inline		 LPSML		GetChild(IDX idx) const					{ return m_arrChildren[idx]; }
	inline const LPSMLARR&	GetArrChildren() const					{ return m_arrChildren; }
	inline		 LPSMLARR&	GetArrChildren()						{ return m_arrChildren; }
	void					SetFncItem(TFncInitItem, TFncSaveItem, TFncReleaseItem, void* data=NULL);

public:
	static int STCALL Compare(const void*, const void*);
	static int STCALL CompareName(const void*, const void*);

private:
	typedef TokenInputStream<false> TokenIStream;

	bool			ParseSection(TokenIStream&, MemFile&);
	bool			ParseValues(MemFile&);
	bool			SaveBracketize(OSTREAM&, const String&, SAVEFLAG) const;
	bool			SaveIntern(OSTREAM&, const String&, SAVEFLAG) const;

private:
	const String	m_strName; // node name
	LPSMLARR		m_arrChildren; // the array with all the sub-nodes
	TFncInitItem	m_fncInitItem; // callback function used to initialize each item
	TFncSaveItem	m_fncSaveItem; // callback function used to save each item
	TFncReleaseItem	m_fncReleaseItem; // callback function used to release each item
	void*			m_fncItemData;
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_SML_H__
