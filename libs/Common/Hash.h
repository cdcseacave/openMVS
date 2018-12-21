////////////////////////////////////////////////////////////////////
// Hash.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_HASH_H__
#define __SEACAVE_HASH_H__


// I N C L U D E S /////////////////////////////////////////////////



// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

/**************************************************************************************
 * StringHash template
 * ---------------
 * compile-time string hash template
 **************************************************************************************/

class StringHash
{
private:
	uint32_t m_val;

	template<size_t N> inline uint32_t _Hash(const char (&str)[N]) const
	{
		typedef const char (&truncated_str)[N-2];
		return str[N-1] + 65599 * (str[N-2] + 65599 * _Hash((truncated_str)str));
	}
	inline uint32_t _Hash(const char (&str)[3]) const { return str[2] + 65599 * (str[1] + 65599 * str[0]); }
	inline uint32_t _Hash(const char (&str)[2]) const { return str[1] + 65599 * str[0]; }

public:
	template <size_t N> StringHash(const char (&str)[N]) { m_val = _Hash(str); }
	inline operator uint32_t() const { return m_val; }
};


/**************************************************************************************
 * cHashTable template
 * ---------------
 * simple hash template (performs relatively well for less than 300 elements)
 **************************************************************************************/

#define KEY_NA		0xFFFF
#define KEY_MAXSIZE	0x1000000

template <typename Type>
class cHashTable
{
public:
	typedef uint32_t		Key;
	typedef uint16_t		Idx;
	typedef uint32_t		Size;
	typedef cList<Type>		Values;
	typedef cList<Key, Key>	Keys;
	typedef cList<Idx, Idx>	Indices;

protected:
	Keys					m_keys;		// Array of unique keys that have been set
	Values					m_values;	// Matching array of unique values that have been set
	Indices					m_indices;	// 1-based index array referencing the two arrays above

public:
	cHashTable() : m_indices(32) { m_indices.Memset(0); }
	~cHashTable() { Release(); }

	inline void Release()
	{
		m_keys.Release();
		m_values.Release();
		m_indices.Release();
	}

	inline bool				IsEmpty()				const	{ return m_indices.IsEmpty(); }
	inline size_t			GetSize()				const	{ return m_keys.GetSize(); }
	inline const Type*		GetBegin()				const	{ return m_values.GetData(); }
	inline const Type*		GetEnd()				const	{ return m_values.GetData()+m_values.GetSize(); }
	inline Type*			GetBegin()						{ return m_values.GetData(); }
	inline Type*			GetEnd()						{ return m_values.GetData()+m_values.GetSize(); }

	inline const Indices&	GetArrIndices()			const	{ return m_indices; }
	inline const Keys&		GetArrKeys()			const	{ return m_keys; }
	inline const Values&	GetArrValues()			const	{ return m_values; }
	inline Values&			GetArrValues()					{ return m_values; }

private:
	inline Size _StaticKeyToID(Key key, Size size)	const	{ return key & (size-1); }
	inline Size _KeyToID(Key key)					const	{ return _StaticKeyToID(key, (Size)m_indices.GetSize()); }
	inline Idx  _IDToIndex(Size id)					const	{ return m_indices[id] - 1; }
	inline Idx  _KeyToIndex(Key key)				const	{ return (m_indices.IsEmpty() ? KEY_NA : _IDToIndex(_KeyToID(key))); }

	// Completely discards then rebuilds all indices based on the current set of keys
	void _RebuildIndices()
	{
		// Clear all current memory
		m_indices.Memset(0);
		// Run through all keys and recreate the indices
		for (Idx i=0; i<m_keys.GetSize(); )
		{
			const Size index = _KeyToID(m_keys[i]);
			m_indices[index] = ++i;
		}
	}

public:
	// Returns a pointer to the value of a key if it exists, 0 otherwise
	inline const Type* Find(Key key) const
	{
		const Idx index = _KeyToIndex(key);
		return (index < m_keys.GetSize() && m_keys[index] == key) ? &m_values[index] : NULL;
	}

	// Non-constant version of the function above
	inline Type* Find(Key key)
	{
		const Idx index = _KeyToIndex(key);
		return (index < m_keys.GetSize() && m_keys[index] == key) ? &m_values[index] : NULL;
	}

	// Checks whether the specified key exists is in the hash
	inline bool Exists(Key key) const
	{
		const Idx index = _KeyToIndex(key);
		return (index < m_keys.GetSize() && m_keys[index] == key);
	}

	// Removes a single entry from the list
	void Delete(Key key)
	{
		const Size id		= _KeyToID(key);
		const Idx  index	= _IDToIndex(id);

		if (index < m_keys.GetSize() && m_keys[index] == key)
		{
			m_keys.RemoveAt(index);
			m_values.RemoveAt(index);
			m_indices[id] = 0;

			// Adjust all the indices that follow this one
			for (Size i=m_indices.GetSize(); i>0; )
			{
				if (m_indices[--i] > index)
				{
					--m_indices[i];
				}
			}
		}
	}

	// Retrieves a value from the hash -- but it will only be valid if it exists, so be careful. In most
	// cases you will either want to use an Exists() check first, or simply use the Find function.
	inline const Type& operator [] (Key key) const
	{
		const Idx index = _KeyToIndex(key);
		return m_values[index];
	}

	// Retrieves a value from the hash, inserting a new one if necessary
	Type& operator [] (Key key)
	{
		// Get the index for this key
		const Idx index = _KeyToIndex(key);

		if (index != KEY_NA)
		{
			// If we found a valid entry, we need to match the actual key
			const Key oldKey = m_keys[index];

			// If the key matches, return the value
			if (oldKey == key)
			{
				return m_values[index];
			}
			else
			{
				// Setting the key was unsuccessful due to another entry colliding with our key;
				// we must expand the indices until we find a set of keys that will match.
				Size newSize = m_indices.GetSize();
				while (newSize < (KEY_MAXSIZE>>1))
				{
					newSize = newSize << 1;
					// Find the next best size for the hash that would make both keys unique
					if (_StaticKeyToID(key, newSize) != _StaticKeyToID(oldKey, newSize))
					{
						m_indices.ResizeExact(newSize);
						_RebuildIndices();
						break;
					}
				}
				// Critical error if we didn't find a working size
				ASSERT(_StaticKeyToID(key, newSize) != _StaticKeyToID(oldKey, newSize));
			}
		}

		// assert the total number of values stored is in Idx range
		ASSERT(m_keys.GetSize() < (Size)((Idx)-1));

		// Append the new key to the end
		m_keys.Insert(key);

		// Add this new entry to the index list using the current array size as the 1-based index
		m_indices[_KeyToID(key)] = (Idx)m_keys.GetSize();

		// Return the value
		return m_values.AddEmpty();
	}

public:
	// Fowler / Noll / Vo (FNV) Hash
	// magic numbers from http://www.isthe.com/chongo/tech/comp/fnv/
	static inline Key HashKeyFNV(const uint8_t* data, Size size)
	{
		Key hash = 2166136261U; //InitialFNV
		for (size_t i = 0; i < size; i++)
		{
			hash = hash ^ (data[i]);	// xor the low 8 bits
			hash = hash * 16777619;		// multiply by the magic number (FNVMultiple)
		}
		return hash;
	}
	// Function that calculates a hash value from a given data
	// tested for random binary data (3 <= size <= 33) and performs VERY bad
	static inline Key HashKeyR5(const uint8_t* data, Size size)
	{
		Key key = 0;
		Key offset = 1357980759;
		for (Size i=0; i<size; ++i, ++data)
		{
			key += (*data & 31) ^ offset;
			key += i & (offset >> 15);
			offset = key ^ (((~offset) >> 7) | (offset << 25));
		}
		return key;
	}
	static inline Key HashKey(const uint8_t* data, Size size) { return HashKeyFNV(data, size); }
	static inline Key HashKey(LPCTSTR sz) { return HashKey((const uint8_t*)sz, (Size)_tcslen(sz)); }
	static inline Key HashKey(const String& str) { return HashKey((const uint8_t*)str.c_str(), (Size)str.size()); }

	// Convenience functions
	inline		 Type*	Find		(LPCTSTR		key)			{ return  Find  ( HashKey(key) );			}
	inline		 Type*	Find		(const String&	key)			{ return  Find  ( HashKey(key) );			}
	inline const Type*	Find		(LPCTSTR		key)	const	{ return  Find  ( HashKey(key) );			}
	inline const Type*	Find		(const String&	key)	const	{ return  Find  ( HashKey(key) );			}
	inline void			Delete		(LPCTSTR		key)			{		  Delete( HashKey(key) );			}
	inline void			Delete		(const String&	key)			{		  Delete( HashKey(key.c_str()) );	}
	inline bool			Exists		(LPCTSTR		key)	const	{ return  Exists( HashKey(key) );			}
	inline bool			Exists		(const String&	key)	const	{ return  Exists( HashKey(key.c_str()) );	}
	inline		 Type&	operator [] (LPCTSTR		key)			{ return (*this)[ HashKey(key) ];			}
	inline		 Type&	operator [] (const String&	key)			{ return (*this)[ HashKey(key.c_str()) ];	}
	inline const Type&	operator [] (LPCTSTR		key)	const	{ return (*this)[ HashKey(key) ];			}
	inline const Type&	operator [] (const String&	key)	const	{ return (*this)[ HashKey(key.c_str()) ];	}
};


/**************************************************************************************
 * cMapWrap template
 * ---------------
 * STL map wrapper
 **************************************************************************************/

template <typename Key, typename Type>
class cMapWrap : public std::unordered_map<Key, Type>
{
public:
	typedef typename std::unordered_map<Key, Type> Base;
	typedef typename Base::const_iterator const_iterator;
	typedef typename Base::iterator iterator;

public:
	cMapWrap() {}
	~cMapWrap() {}

	inline void				Release()						{ this->clear(); }
	inline bool				IsEmpty()				const	{ return this->empty(); }
	inline size_t			GetSize()				const	{ return this->size(); }
	inline const_iterator	GetBegin()				const	{ return this->begin(); }
	inline const_iterator	GetEnd()				const	{ return this->end(); }
	inline iterator			GetBegin()						{ return this->begin(); }
	inline iterator			GetEnd()						{ return this->end(); }
	inline const_iterator	Find(const Key& key)	const	{ return this->find(key); }
	inline iterator			Find(const Key& key)			{ return this->find(key); }
	inline iterator			Insert(const Key& key, bool& bExisted) { return Insert(key, Type(), bExisted); }
	inline iterator			Insert(const Key& key, const Type& val, bool& bExisted) {
		std::pair<iterator,bool> ret = this->insert(std::pair<Key,Type>(key, val));
		bExisted = !ret.second;
		return ret.first;
	}
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_HASH_H__
