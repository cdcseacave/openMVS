////////////////////////////////////////////////////////////////////
// SML.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "SML.h"

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////

#define SML_AUTOVALUES_OFF				0
#define SML_AUTOVALUES_ON				1
#ifndef SML_AUTOVALUES
#define SML_AUTOVALUES					SML_AUTOVALUES_ON
#endif

#define SML_SECTIONOPENBRACKET			_T("{")
#define SML_SECTIONCLOSEBRACKET			_T("}")
#define SML_NAMEOPENBRACKET				_T("[")
#define SML_NAMECLOSEBRACKET			_T("]")
#define SML_NAMETOKEN					_T("=")
#define SML_VALUETOKEN					_T("\n")
#define SML_INDENT						_T("\t")

#define SML_TOKEN_SECTIONOPENBRACKET	SML_SECTIONOPENBRACKET[0]
#define SML_TOKEN_SECTIONCLOSEBRACKET	SML_SECTIONCLOSEBRACKET[0]
#define SML_TOKEN_NAMEOPENBRACKET		SML_NAMEOPENBRACKET[0]
#define SML_TOKEN_NAMECLOSEBRACKET		SML_NAMECLOSEBRACKET[0]
#define SML_TOKEN_NAMETOKEN				SML_NAMETOKEN[0]
#define SML_TOKEN_VALUETOKEN			SML_VALUETOKEN[0]
#define SML_TOKEN_INDENT				SML_INDENT[0]
#define SML_TOKEN_IGNORECHARS			_T("\n\r\t ")


// S T R U C T S ///////////////////////////////////////////////////

/*-----------------------------------------------------------*
 * SML class implementation                                  *
 *-----------------------------------------------------------*/

/**
 * Constructor
 */
SML::SML(const String& name)
	:
	m_strName(name),
	m_fncInitItem(NULL),
	m_fncSaveItem(NULL),
	m_fncReleaseItem(NULL),
	m_fncItemData(NULL)
{
}

/**
 * Destructor
 */
SML::~SML()
{
	Release();
}
/*----------------------------------------------------------------*/

/**
 * Released all used memory for this nod and its children
 */
void SML::Release()
{
	m_arrChildren.ReleaseDelete();
	if (m_fncReleaseItem != NULL) {
		for (SMLITEMMAP::iterator it=GetBegin(); it!=GetEnd(); ++it)
			m_fncReleaseItem(it->second, m_fncItemData);
	}
	SMLITEMMAP::Release();
}
/*----------------------------------------------------------------*/


/**
 * Load the values from a file;
 * all children are generated if they do not exist
 */
bool SML::Load(const String& fileName)
{
	File oStream(fileName, File::READ, File::OPEN);
	if (!oStream.isOpen())
		return false;
	return Load(oStream);
}
bool SML::Load(ISTREAM& oStream)
{
	// create the section token filter and mem-file
	MemFile memFile;
	TokenIStream filter(&oStream);
	filter.setTrimTokens(SML_TOKEN_IGNORECHARS);
	// and parse section
	return ParseSection(filter, memFile);
}
bool SML::ParseSection(TokenIStream& filter, MemFile& memFile)
{
	while (true) {
		// find first section start (NameOpenBracket)
		filter.setToken(SML_TOKEN_NAMEOPENBRACKET);
		filter.readLine(memFile);

		// parse values before the new section or end of section
		const size_f_t posMemFile = memFile.getPos();
		MemFile valuesMemFile;
		TokenIStream sectionFilter(&memFile, SML_TOKEN_SECTIONCLOSEBRACKET);
		const size_f_t lenValues = sectionFilter.readLine(valuesMemFile);
		if (ParseValues(valuesMemFile) == false)
			return false; // Parse Error: invalid values
		ASSERT(valuesMemFile.getSize() == 0);

		// if end of section found, return
		if (!sectionFilter.isEOS()) {
			// if a name open bracket was found before the EOS,
			// then restore it for the parent next ParseSection()
			memFile.setPos(posMemFile+(lenValues+1)*sizeof(TCHAR));
			if (!filter.isEOS())
				filter.restoreToken();
			break;
		}
		else {
			ASSERT(memFile.getSize()-posMemFile == lenValues*(size_f_t)sizeof(TCHAR));
			ASSERT(posMemFile == 0 || memFile.getSize()-posMemFile == memFile.getSizeLeft() || memFile.getSizeLeft() == 0);
			memFile.setSize(0);
		}

		// if no more data, return
		if (filter.isEOS())
			break;

		// parse child section name
		filter.setToken(SML_TOKEN_NAMECLOSEBRACKET);
		const size_t lenName = filter.readLine(memFile);
		ASSERT(!filter.isEOS());
		if (lenName == 0)
			return false; // Parse Error: invalid section name
		const String strChildName((LPCTSTR)memFile.getData(), lenName);
		memFile.growSize(-((size_f_t)(lenName*sizeof(TCHAR))));
		ASSERT(memFile.getSize() == 0);
		// create the child with the given name
		const IDX idxChild = CreateChildUnique(strChildName);
		LPSML const pChild = m_arrChildren[idxChild];
		pChild->SetFncItem(m_fncInitItem, m_fncSaveItem, m_fncReleaseItem, m_fncItemData);

		// parse child section
		filter.setToken(SML_TOKEN_SECTIONOPENBRACKET);
		filter.readLine(memFile);
		filter.trimBackLine(memFile);
		ASSERT(memFile.getSize() == 0);
		if (pChild->ParseSection(filter, memFile) == false)
			return false;
	}
	return true;
}
bool SML::ParseValues(MemFile& valuesMemFile)
{
	if (valuesMemFile.getSize() == 0)
		return true;
	// loop through all name/value pairs
	MemFile memLine;
	TokenIStream filterLine(&valuesMemFile);
	filterLine.setTrimTokens(SML_TOKEN_IGNORECHARS);
	filterLine.setToken(SML_TOKEN_VALUETOKEN);
	MemFile memValue;
	TokenIStream filterValue(&memLine);
	filterValue.setTrimTokens(SML_TOKEN_IGNORECHARS);
	filterValue.setToken(SML_TOKEN_NAMETOKEN);
	do {
		// parse value name and value
		filterLine.readLine(memLine);
		filterLine.trimBackLine(memLine);
		filterLine.trimFrontLine(memLine);
		if (memLine.isEOF())
			continue; // empty line, return
		// parse value name
		const size_t lenNameValueReal = (size_t)memLine.getSizeLeft();
		const size_t lenName = filterValue.readLine(memValue);
		#if SML_AUTOVALUES == SML_AUTOVALUES_ON
		String szName;
		if (filterValue.trimBackLine(memValue) == lenName || lenNameValueReal == lenName) {
			// no name found, auto generate the name
			szName = _T("Item") + String::ToString(size());
		} else {
			// read the name
			szName = (LPCTSTR)memValue.getData();
			memValue.setSize(0);
		}
		#else
		filterValue.trimBackLine(memValue);
		String szName = (LPCTSTR)memValue.getData();
		ASSERT(!filterValue.isEOS() && !szName.IsEmpty());
		if (filterValue.isEOS() || szName.IsEmpty()) {
			memValue.setSize(0);
			memLine.setSize(0);
			filterValue.setPos(0);
			continue; // Parse Error: invalid syntax ('=' not found)
		}
		ASSERT(szName.size() == memValue.getSizeLeft());
		memValue.setSize(0);
		#endif
		SMLVALUE& val = operator[](szName);
		// parse value
		filterValue.read(memValue);
		LPCTSTR szValue = filterValue.trimFrontLine(memValue);
		val.val = szValue;
		ASSERT((size_f_t)_tcslen(szValue) == memValue.getSizeLeft());
		memValue.setSize(0);
		memLine.setSize(0);
		filterValue.setPos(0);
	} while (!filterLine.isEOS());
	// all file processed, safe to reset it to 0
	valuesMemFile.setSize(0);
	return true;
}
/*----------------------------------------------------------------*/


/**
 * Write to a file the values of this node and its children.
 * Set to false the second parameter in order not to save the empty children.
 */
bool SML::Save(const String& fileName, SAVEFLAG flags) const
{
	File oStream(fileName, File::WRITE, File::CREATE | File::TRUNCATE);
	if (!oStream.isOpen())
		return false;
	return Save(oStream, flags);
}
bool SML::Save(OSTREAM& oStream, SAVEFLAG flags) const
{
	// save all values and all children
	return SaveIntern(oStream, _T(""), flags);
}
bool SML::SaveBracketize(OSTREAM& oStream, const String& strIndent, SAVEFLAG flags) const
{
	// save its name
	oStream.print(_T("%s" SML_NAMEOPENBRACKET "%s" SML_NAMECLOSEBRACKET "\n"), strIndent.c_str(), m_strName.c_str());
	oStream.print(_T("%s" SML_SECTIONOPENBRACKET "\n"), strIndent.c_str());
	// save all values and all children
	SaveIntern(oStream, strIndent+SML_TOKEN_INDENT, flags);
	// close bracket
	oStream.print(_T("%s" SML_SECTIONCLOSEBRACKET "\n"), strIndent.c_str());
	return true;
}
bool SML::SaveIntern(OSTREAM& oStream, const String& strIndent, SAVEFLAG flags) const
{
	const Flags& flgs = (const Flags&)flags;
	if (flgs.isAnySet(SORT | SORTINV)) {
		// sort values alphabetically and save them
		StringArr lines(0, GetSize());
		for (SMLITEMMAP::const_iterator item=GetBegin(); item!=GetEnd(); ++item)
			if (!m_fncSaveItem || m_fncSaveItem((*item).second, m_fncItemData))
				lines.InsertSort(String::FormatString(_T("%s%s " SML_NAMETOKEN " %s" SML_VALUETOKEN), strIndent.c_str(), (*item).first.c_str(), (*item).second.val.c_str()),
					(flgs.isAnySet(SORT) ? String::CompareAlphabetically : String::CompareAlphabeticallyInv));
		FOREACH(l, lines)
			oStream.print(lines[l]);
	} else {
		// save all values directly
		for (SMLITEMMAP::const_iterator item=GetBegin(); item!=GetEnd(); ++item)
			if (!m_fncSaveItem || m_fncSaveItem((*item).second, m_fncItemData))
				oStream.print(_T("%s%s " SML_NAMETOKEN " %s" SML_VALUETOKEN), strIndent.c_str(), (*item).first.c_str(), (*item).second.val.c_str());
	}
	// save now all children
	bool bFirst = IsEmpty();
	FOREACH(i, m_arrChildren) {
		const SML* pSML = m_arrChildren[i];
		// skip empty children
		if (!flgs.isSet(SAVEEMPTY) && pSML->IsEmpty() && pSML->m_arrChildren.IsEmpty())
			continue;
		// insert a new line to separate from the above section (only for visual aspect)
		if (bFirst)
			bFirst = false;
		else
			oStream.print(_T("\n"));
		// save child
		pSML->SaveBracketize(oStream, strIndent, flags);
	}
	return true;
}
/*----------------------------------------------------------------*/


/**
 * Create and insert a new child
 */
IDX SML::CreateChild(const String& strChildName)
{
	return InsertChild(new SML(strChildName));
}
IDX SML::CreateChildUnique(const String& strChildName)
{
	return InsertChildUnique(new SML(strChildName));
}
/*----------------------------------------------------------------*/


/**
 * Insert a new child
 */
IDX SML::InsertChild(const LPSML pSML)
{
	return m_arrChildren.InsertSort(pSML, SML::Compare);
}
IDX SML::InsertChildUnique(const LPSML pSML)
{
	const std::pair<IDX,bool> res(m_arrChildren.InsertSortUnique(pSML, SML::Compare));
	if (res.second)
		delete pSML;
	return res.first;
}
/*----------------------------------------------------------------*/


/**
 * Remove an existing child
 */
void SML::RemoveChild(IDX idx)
{
	m_arrChildren.RemoveAtMove(idx);
}
/*----------------------------------------------------------------*/

/**
 * Remove and destroy an existing child
 */
void SML::DestroyChild(IDX idx)
{
	delete m_arrChildren[idx];
	RemoveChild(idx);
}
/*----------------------------------------------------------------*/


/**
 * Retrieve an child by its name; NO_INDEX if unexistent
 */
IDX SML::GetChild(const String& name) const
{
	return m_arrChildren.FindFirst(&name, SML::CompareName);
}
/*----------------------------------------------------------------*/


/**
 * Retrieve an item; NULL if unexistent
 */
const SMLVALUE* SML::GetValue(const String& key) const
{
	const_iterator it = Find(key);
	if (it == GetEnd())
		return NULL;
	return &(it->second);
}
/*----------------------------------------------------------------*/


/**
 * Insert or retrieve an item; initializa it if necesary
 */
SMLVALUE& SML::GetValue(const String& key)
{
	bool bExisted;
	SMLVALUE& val = (*Insert(key, bExisted)).second;
	if (!bExisted && m_fncInitItem != NULL)
		m_fncInitItem(key, val, m_fncItemData);
	return val;
}
/*----------------------------------------------------------------*/


/**
 * Reset items' init and release functions
 */
void SML::SetFncItem(TFncInitItem fncInit, TFncSaveItem fncSave, TFncReleaseItem fncRelease, void* data)
{
	m_fncInitItem = fncInit;
	m_fncSaveItem = fncSave;
	m_fncReleaseItem = fncRelease;
	m_fncItemData = data;
}
/*----------------------------------------------------------------*/


/**
 * Compare two SML nodes by name
 */
int STCALL SML::Compare(const void* l, const void* r)
{
	return _tcscmp((*((const SML**)l))->GetName(), (*((const SML**)r))->GetName());
}
/*----------------------------------------------------------------*/

/**
 * Compare a name and a SML node name
 */
int STCALL SML::CompareName(const void* l, const void* r)
{
	return _tcscmp((*((const SML**)l))->GetName(), ((const String*)r)->c_str());
}
/*----------------------------------------------------------------*/
