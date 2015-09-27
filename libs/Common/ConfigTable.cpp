////////////////////////////////////////////////////////////////////
// ConfigTable.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "ConfigTable.h"

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

/*-----------------------------------------------------------*
 * CConfigTable class implementation                         *
 *-----------------------------------------------------------*/

/**
 * Constructor
 */
CConfigTable::CConfigTable(const String& name) : m_oSML(name), m_pParent(NULL)
{
	m_oSML.SetFncItem(ItemInitData, ItemSaveData, ItemReleaseData);
}

/**
 * Destructor
 */
CConfigTable::~CConfigTable()
{
	if (m_pParent)
		m_pParent->RemoveChild(*this);
	Release();
}
/*----------------------------------------------------------------*/

/**
 * Released all used memory
 */
void CConfigTable::Release()
{
	m_oSML.Release();
}
/*----------------------------------------------------------------*/


/**
 * Create a new child
 */
void CConfigTable::Insert(const String& name)
{
	SML* const pChildSML = new SML(name);
	pChildSML->SetFncItem(ItemInitData, ItemSaveData, ItemReleaseData);
	const IDX idx = m_oSML.InsertChild(pChildSML);
	// if child already exists, delete created node
	if (m_oSML.GetArrChildren()[idx] != pChildSML)
		delete pChildSML;
}
/*----------------------------------------------------------------*/


/**
 * Remove an existing child
 */
void CConfigTable::Remove(const String& name)
{
	m_oSML.DestroyChild(name);
}
/*----------------------------------------------------------------*/


/**
 * Get the config table for the given child
 */
const SML& CConfigTable::GetConfig(const String& name) const
{
	const LPSMLARR& arrSML = m_oSML.GetArrChildren();
	const IDX idx = arrSML.FindFirst(&name, SML::CompareName);
	ASSERT(idx != LPSMLARR::NO_INDEX);
	return *arrSML[idx];
} // GetConfig
SML& CConfigTable::GetConfig(const String& name)
{
	const LPSMLARR& arrSML = m_oSML.GetArrChildren();
	const IDX idx = arrSML.FindFirst(&name, SML::CompareName);
	ASSERT(idx != LPSMLARR::NO_INDEX);
	return *arrSML[idx];
} // GetConfig
/*----------------------------------------------------------------*/


/**
 * Load the configuration from a file.
 */
bool CConfigTable::Load(const String& f)
{
	return m_oSML.Load(f);
} // Load
bool CConfigTable::Load(ISTREAM& oStream)
{
	return m_oSML.Load(oStream);
} // Load
/**
 * Write to a file the values of this node and its children.
 * Set to false the second parameter in order not to save the empty children.
 */
bool CConfigTable::Save(const String& f, SML::SAVEFLAG flags) const
{
	return m_oSML.Save(f, flags);
} // Save
bool CConfigTable::Save(OSTREAM& oStream, SML::SAVEFLAG flags) const
{
	return m_oSML.Save(oStream, flags);
} // Save
/*----------------------------------------------------------------*/


/**
 * Create and initialize a config item for the given SML entry.
 */
void STCALL CConfigTable::ItemInitData(const String& key, SMLVALUE& val, void*)
{
	CFGITEM* pItem = new CFGITEM;
	pItem->name = key;
	val.data = pItem;
} // ItemInitData
/*----------------------------------------------------------------*/

/**
 * Save a config item for the given SML entry. Return false if this item should not be saved.
 */
bool STCALL CConfigTable::ItemSaveData(const SMLVALUE& val, void*)
{
	const CFGITEM& item = *((const CFGITEM*)val.data);
	return !item.state.isSet(CFGITEM::TEMP);
} // ItemSaveData
/*----------------------------------------------------------------*/

/**
 * Release and destroy the config item for the given SML entry.
 */
void STCALL CConfigTable::ItemReleaseData(SMLVALUE& val, void*)
{
	CFGITEM* pItem = (CFGITEM*)val.data;
	val.data = NULL;
	delete pItem;
} // ItemReleaseData
/*----------------------------------------------------------------*/
