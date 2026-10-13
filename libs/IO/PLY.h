////////////////////////////////////////////////////////////////////
// PLY.h
//
// Copyright 2023 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)
//
// PLY reader and writer.
//
// A PLY file holds a sequence of elements (vertex, face, ...); each element is a table of
// records whose columns are its properties: scalars, or lists of scalars preceded by their
// count. The values are read straight into, and written straight from, the caller's arrays:
// a Binding names a property and gives the memory of its value in the first record and the
// stride to the next record, so no intermediate record structure is involved.
//
// Reading:
//   PLY ply;
//   if (!ply.Open(fileName)) ...
//   for (size_t e = 0; e < ply.GetElements().size(); ++e)    // elements are read in file order,
//       if (ply.GetElements()[e].name == "vertex")            // the ones not read are skipped
//           ok = ply.ReadElement(e, {PLY::Scalar("x", &points[0].x, sizeof(Point)), ...});
// Writing:
//   PLY ply;
//   ply.AddElement("vertex", points.size(), {PLY::Scalar("x", &points[0].x, sizeof(Point)), ...});
//   ok = ply.Save(fileName);

#ifndef __SEACAVE_PLY_H__
#define __SEACAVE_PLY_H__


// S T R U C T S ///////////////////////////////////////////////////

class IO_API PLY
{
public:
	enum FileType {
		ASCII,
		BINARY_LE,
		BINARY_BE, // read only
	};
	enum DataType : uint8_t {
		NoType, Int8, Int16, Int32, Uint8, Uint16, Uint32, Float32, Float64
	};

	// a property as declared in the file header
	struct Property {
		std::string name;
		DataType type;      // type of the scalar, or of the list items
		DataType countType; // NoType for a scalar, else the type of the list count
		bool IsList() const { return countType != NoType; }
	};
	// an element as declared in the file header
	struct Element {
		std::string name;
		size_t count;       // number of records
		std::vector<Property> props;
		const Property* FindProperty(const char* name) const {
			for (const Property& prop: props)
				if (prop.name == name)
					return &prop;
			return NULL;
		}
		bool HasProperties(std::initializer_list<const char*> names) const {
			for (const char* name: names)
				if (FindProperty(name) == NULL)
					return false;
			return true;
		}
	};

	// the memory a property is read into or written from
	struct Binding {
		typedef void (*SetList)(void* lists, size_t idx, uint32_t numItems, const void* items);
		typedef uint32_t (*GetList)(const void* lists, size_t idx, const void*& items);
		const char* name;
		DataType type;      // type in memory of the scalar, or of the list items
		DataType fileType;  // writing: type in the file of the scalar, or of the list items
		DataType countType; // NoType for a scalar; writing a list: type in the file of its count
		uint32_t numItems;  // fixed-size list: items per record (reading any other count fails); 0: variable-size list
		void* data;         // scalar or fixed-size list: the value(s) of the first record; variable-size list: the lists
		size_t stride;      // scalar or fixed-size list: bytes from one record's value(s) to the next record's
		SetList setList;    // variable-size list, reading: stores the items of record idx (packed, possibly unaligned)
		GetList getList;    // variable-size list, writing: returns the items of record idx
	};
	typedef std::vector<Binding> Bindings;

	template <typename T>
	static constexpr DataType TypeOf() {
		if constexpr (std::is_same<T,int8_t>::value) return Int8;
		else if constexpr (std::is_same<T,int16_t>::value) return Int16;
		else if constexpr (std::is_same<T,int32_t>::value) return Int32;
		else if constexpr (std::is_same<T,uint8_t>::value) return Uint8;
		else if constexpr (std::is_same<T,uint16_t>::value) return Uint16;
		else if constexpr (std::is_same<T,uint32_t>::value) return Uint32;
		else if constexpr (std::is_same<T,float>::value) return Float32;
		else if constexpr (std::is_same<T,double>::value) return Float64;
		else static_assert(sizeof(T) == 0, "type not supported by PLY");
	}
	// a scalar property: its value is at first for record 0, then every stride bytes;
	// fileType is the type written in the file (reading converts from the file's type)
	template <typename T>
	static Binding Scalar(const char* name, const T* first, size_t stride, DataType fileType=TypeOf<T>()) {
		return Binding{name, TypeOf<T>(), fileType, NoType, 0, const_cast<T*>(first), stride, NULL, NULL};
	}
	// a list property of numItems items per record, stored contiguously from first for record 0, then every stride bytes
	template <typename T>
	static Binding List(const char* name, const T* first, uint32_t numItems, size_t stride, DataType countType=Uint8) {
		ASSERT(numItems > 0);
		return Binding{name, TypeOf<T>(), TypeOf<T>(), countType, numItems, const_cast<T*>(first), stride, NULL, NULL};
	}
	// a list property of any size per record, kept in lists[idx] (a container with resize(), data() and size())
	template <typename ARR>
	static Binding Lists(const char* name, const ARR& lists, DataType countType=Uint8) {
		typedef typename std::decay<decltype(*lists[0].data())>::type T;
		return Binding{name, TypeOf<T>(), TypeOf<T>(), countType, 0, const_cast<ARR*>(&lists), 0, &SetItems<ARR,T>, &GetItems<ARR,T>};
	}

	PLY();
	~PLY();

	// reading: parse the header; the elements are then read in file order
	bool Open(const std::string& fileName);
	FileType GetFileType() const { return fileType; }
	const std::vector<Element>& GetElements() const { return elements; }
	const std::vector<std::string>& GetComments() const { return comments; }
	// read every record of element idx into the bound memory, skipping the properties not bound and
	// any earlier element not read; fails on a bound property the element lacks, on a fixed-size list
	// of another size and on a truncated or malformed file
	bool ReadElement(size_t idx, const Bindings& bindings);

	// writing: describe the elements in file order (their memory must stay valid until Save;
	// an element without records may bind NULL, or declare no property at all)
	void AddComment(const std::string& comment) { comments.emplace_back(comment); }
	void AddElement(const char* name, size_t count, Bindings bindings);
	bool Save(const std::string& fileName, FileType type=BINARY_LE, bool legacyTypeNames=false) const;

protected:
	struct Column;

	template <typename ARR, typename T>
	static void SetItems(void* lists, size_t idx, uint32_t numItems, const void* items) {
		auto& list = (*static_cast<ARR*>(lists))[static_cast<decltype(std::declval<ARR&>().size())>(idx)];
		list.resize(numItems);
		if (numItems)
			memcpy(list.data(), items, sizeof(T)*numItems);
	}
	template <typename ARR, typename T>
	static uint32_t GetItems(const void* lists, size_t idx, const void*& items) {
		const auto& list = (*static_cast<const ARR*>(lists))[static_cast<decltype(std::declval<const ARR&>().size())>(idx)];
		items = list.data();
		return static_cast<uint32_t>(list.size());
	}

	bool Fill(size_t numBytes);
	bool ReadLine(std::string& line);
	bool ParseHeader();
	bool ReadBinary(const Element&, std::vector<Column>&);
	bool ReadAscii(const Element&, std::vector<Column>&);
	bool NextToken(const char*& token, size_t& len);

protected:
	// reading
	SEACAVE::File file;
	std::vector<uint8_t> buffer;    // read-ahead buffer: the unread bytes are [bufferPos, bufferEnd)
	size_t bufferPos, bufferEnd;
	SEACAVE::size_f_t fileLeft;     // bytes of the file not yet in the buffer
	FileType fileType;
	size_t nextElement;             // first element not read yet
	std::vector<Element> elements;
	std::vector<std::string> comments;

	// writing
	struct OutElement {
		std::string name;
		size_t count;
		Bindings bindings;
	};
	std::vector<OutElement> outElements;
};
/*----------------------------------------------------------------*/

#endif // __SEACAVE_PLY_H__
