////////////////////////////////////////////////////////////////////
// PLY.cpp
//
// Copyright 2023 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "PLY.h"
#include <charconv>

using namespace SEACAVE;


// S T R U C T S ///////////////////////////////////////////////////

// the host is little-endian, as on every platform OpenMVS builds for: big-endian files are swapped on read
namespace {

const char* const typeNames[] = {"", "int8", "int16", "int32", "uint8", "uint16", "uint32", "float32", "float64"};
const char* const oldTypeNames[] = {"", "char", "short", "int", "uchar", "ushort", "uint", "float", "double"};
constexpr uint32_t typeSizes[] = {0, 1, 2, 4, 1, 2, 4, 4, 8};

constexpr size_t BUFFER_SIZE = 4u<<20;    // read-ahead and write buffer
constexpr size_t MAX_IO_CHUNK = 1u<<30;   // largest single file read/write (Win32 takes 32-bit sizes)
constexpr size_t MAX_HEADER_LINE = 1u<<20;
constexpr size_t MAX_TOKEN = 64;          // longest ascii value

inline bool IsBlank(uint8_t c) { return c == ' ' || c == '\t' || c == '\r'; }
inline bool IsSpace(uint8_t c) { return IsBlank(c) || c == '\n'; }
inline bool IsInteger(PLY::DataType t) { return t >= PLY::Int8 && t <= PLY::Uint32; }

PLY::DataType ParseType(const std::string& name) {
	for (int t = PLY::Int8; t <= PLY::Float64; ++t)
		if (name == typeNames[t] || name == oldTypeNames[t])
			return static_cast<PLY::DataType>(t);
	return PLY::NoType;
}

// call f with a value of the C++ type of t
template <typename F>
auto Dispatch(PLY::DataType t, F&& f) {
	switch (t) {
	case PLY::Int8: return f(int8_t());
	case PLY::Int16: return f(int16_t());
	case PLY::Int32: return f(int32_t());
	case PLY::Uint8: return f(uint8_t());
	case PLY::Uint16: return f(uint16_t());
	case PLY::Uint32: return f(uint32_t());
	case PLY::Float32: return f(float());
	default: ASSERT(t == PLY::Float64); return f(double());
	}
}

template <typename T>
inline T ByteSwap(T v) {
	uint8_t b[sizeof(T)];
	memcpy(b, &v, sizeof(T));
	std::reverse(b, b+sizeof(T));
	memcpy(&v, b, sizeof(T));
	return v;
}

// convert n packed values of type S at src to type D at dst, swapping the bytes of the source if SWAP
typedef void (*ConvertFn)(const uint8_t* src, uint8_t* dst, size_t n);
template <typename S, typename D, bool SWAP>
void ConvertValues(const uint8_t* src, uint8_t* dst, size_t n) {
	for (size_t i = 0; i < n; ++i) {
		S s;
		memcpy(&s, src+i*sizeof(S), sizeof(S));
		if constexpr (SWAP && sizeof(S) > 1)
			s = ByteSwap(s);
		const D d(static_cast<D>(s));
		memcpy(dst+i*sizeof(D), &d, sizeof(D));
	}
}
// the conversion between two types, or NULL when the bytes are copied as they are
ConvertFn Converter(PLY::DataType from, PLY::DataType to, bool swap) {
	if (from == to && (!swap || typeSizes[from] == 1))
		return NULL;
	return Dispatch(from, [to, swap](auto s) {
		typedef decltype(s) S;
		return Dispatch(to, [swap](auto d) -> ConvertFn {
			typedef decltype(d) D;
			return swap ? &ConvertValues<S,D,true> : &ConvertValues<S,D,false>;
		});
	});
}

// the count of a list, stored as the integer type t; false if it is negative or does not fit 32 bits
inline bool DecodeCount(const uint8_t* p, PLY::DataType t, bool swap, uint32_t& count) {
	const int64_t value(Dispatch(t, [p, swap](auto v) -> int64_t {
		typedef decltype(v) T;
		memcpy(&v, p, sizeof(T));
		if (swap)
			v = ByteSwap(v);
		return static_cast<int64_t>(v);
	}));
	if (value < 0 || value > (int64_t)UINT32_MAX)
		return false;
	count = static_cast<uint32_t>(value);
	return true;
}

bool ReadChunked(File& file, uint8_t* dst, size_t size) {
	while (size) {
		const size_t chunk(std::min(size, MAX_IO_CHUNK));
		if (file.read(dst, chunk) != chunk)
			return false;
		dst += chunk;
		size -= chunk;
	}
	return true;
}
bool WriteChunked(File& file, const uint8_t* src, size_t size) {
	while (size) {
		const size_t chunk(std::min(size, MAX_IO_CHUNK));
		if (file.write(src, chunk) != chunk)
			return false;
		src += chunk;
		size -= chunk;
	}
	return true;
}

// buffered file output
struct Writer {
	File file;
	std::vector<uint8_t> buffer;
	size_t size;
	bool ok;
	explicit Writer(const std::string& fileName)
		: file(fileName.c_str(), File::WRITE, File::CREATE | File::TRUNCATE), buffer(BUFFER_SIZE), size(0), ok(file.isOpen()) {}
	// room for n more bytes, flushing the buffer first if needed; Commit the bytes used
	uint8_t* Reserve(size_t n) {
		if (size + n > buffer.size()) {
			Flush();
			if (n > buffer.size())
				buffer.resize(n);
		}
		return buffer.data() + size;
	}
	void Commit(size_t n) { size += n; }
	void Write(const void* data, size_t n) {
		if (n < buffer.size()) {
			memcpy(Reserve(n), data, n);
			size += n;
			return;
		}
		Flush();
		ok = ok && WriteChunked(file, static_cast<const uint8_t*>(data), n);
	}
	bool Flush() {
		if (size) {
			ok = ok && WriteChunked(file, buffer.data(), size);
			size = 0;
		}
		return ok;
	}
};

// the shortest %g text reading back as v
template <typename T>
int FormatReal(char* s, T v) {
	for (int digits = std::numeric_limits<T>::digits10; ; ++digits) {
		const int len(snprintf(s, 32, "%.*g", digits, (double)v));
		if (digits >= std::numeric_limits<T>::max_digits10)
			return len;
		if constexpr (std::is_same<T,float>::value) {
			if (strtof(s, NULL) == v)
				return len;
		} else {
			if (strtod(s, NULL) == v)
				return len;
		}
	}
}
// the text of the value of type t at v
int FormatValue(char* s, PLY::DataType t, const uint8_t* v) {
	return Dispatch(t, [s, v](auto x) -> int {
		typedef decltype(x) T;
		memcpy(&x, v, sizeof(T));
		if constexpr (std::is_floating_point<T>::value)
			return FormatReal(s, x);
		else
			return static_cast<int>(std::to_chars(s, s+32, static_cast<long long>(x)).ptr - s);
	});
}

// a binding prepared for writing: its file encoding
struct OutField {
	const PLY::Binding* binding;
	uint32_t countSize;   // list: bytes of its count, else 0
	uint8_t count[8];     // fixed-size list: its count, encoded
	uint32_t bytes;       // fixed size: bytes of the value(s) in the file
	uint32_t itemSize;    // bytes of a value in the file
	ConvertFn convert;    // memory to file type, NULL: the bytes are copied
	ConvertFn convertCount; // variable-size list: count to its file type, NULL: copied
};
std::vector<OutField> PrepareFields(const PLY::Bindings& bindings) {
	std::vector<OutField> fields;
	fields.reserve(bindings.size());
	for (const PLY::Binding& b: bindings) {
		OutField f;
		f.binding = &b;
		f.itemSize = typeSizes[b.fileType];
		f.convert = Converter(b.type, b.fileType, false);
		f.convertCount = NULL;
		if (b.countType == PLY::NoType) {
			f.countSize = 0;
			f.bytes = f.itemSize;
		} else {
			ASSERT(IsInteger(b.countType));
			f.countSize = typeSizes[b.countType];
			f.bytes = b.numItems*f.itemSize;
			f.convertCount = Converter(PLY::Uint32, b.countType, false);
			if (f.convertCount)
				f.convertCount(reinterpret_cast<const uint8_t*>(&b.numItems), f.count, 1);
			else
				memcpy(f.count, &b.numItems, sizeof(uint32_t));
		}
		// a plain copy of memory adjacent to the previous plain copy, at the same stride, joins it
		if (!fields.empty()) {
			OutField& prev = fields.back();
			const PLY::Binding& pb = *prev.binding;
			if (f.countSize == 0 && prev.countSize == 0 && f.convert == NULL && prev.convert == NULL &&
				static_cast<const uint8_t*>(pb.data) + prev.bytes == static_cast<const uint8_t*>(b.data) && pb.stride == b.stride) {
				prev.bytes += f.bytes;
				continue;
			}
		}
		fields.push_back(f);
	}
	return fields;
}

// the largest list count the integer type t holds
inline uint64_t MaxCount(PLY::DataType t) {
	return Dispatch(t, [](auto v) -> uint64_t { return (uint64_t)std::numeric_limits<decltype(v)>::max(); });
}

bool WriteBinary(Writer& out, size_t count, const PLY::Bindings& bindings) {
	if (count == 0)
		return out.ok;
	const std::vector<OutField> fields(PrepareFields(bindings));
	// a single plain copy of the whole record, laid out as in memory: write the memory as it is
	if (fields.size() == 1 && fields[0].countSize == 0 && fields[0].convert == NULL && fields[0].bytes == fields[0].binding->stride) {
		out.Write(fields[0].binding->data, count*fields[0].bytes);
		return out.ok;
	}
	for (size_t i = 0; i < count; ++i) {
		for (const OutField& f: fields) {
			const PLY::Binding& b = *f.binding;
			if (b.countType != PLY::NoType && b.numItems == 0) {
				// variable-size list
				const void* items;
				const uint32_t n(b.getList(b.data, i, items));
				ASSERT(n <= MaxCount(b.countType));
				uint8_t* const p(out.Reserve(f.countSize + (size_t)n*f.itemSize));
				if (f.convertCount)
					f.convertCount(reinterpret_cast<const uint8_t*>(&n), p, 1);
				else
					memcpy(p, &n, sizeof(uint32_t));
				if (f.convert)
					f.convert(static_cast<const uint8_t*>(items), p+f.countSize, n);
				else if (n)
					memcpy(p+f.countSize, items, (size_t)n*f.itemSize);
				out.Commit(f.countSize + (size_t)n*f.itemSize);
				continue;
			}
			uint8_t* const p(out.Reserve(f.countSize + f.bytes));
			memcpy(p, f.count, f.countSize);
			const uint8_t* const src(static_cast<const uint8_t*>(b.data) + i*b.stride);
			if (f.convert)
				f.convert(src, p+f.countSize, b.countType == PLY::NoType ? 1 : b.numItems);
			else
				memcpy(p+f.countSize, src, f.bytes);
			out.Commit(f.countSize + f.bytes);
		}
	}
	return out.ok;
}

bool WriteAscii(Writer& out, size_t count, const PLY::Bindings& bindings) {
	std::vector<ConvertFn> converts;
	for (const PLY::Binding& b: bindings)
		converts.push_back(Converter(b.type, b.fileType, false));
	char text[64];
	uint8_t value[8];
	// the values of a record are separated by a space
	const auto Put = [&](size_t len, bool first) {
		memcpy(out.Reserve(len+1), text + first, len + !first);
		out.Commit(len + !first);
	};
	for (size_t i = 0; i < count; ++i) {
		bool first(true);
		for (size_t p = 0; p < bindings.size(); ++p) {
			const PLY::Binding& b(bindings[p]);
			const uint8_t* src(static_cast<const uint8_t*>(b.data) + i*b.stride);
			size_t n(1);
			if (b.countType != PLY::NoType) {
				const void* items(src);
				n = b.numItems ? b.numItems : b.getList(b.data, i, items);
				ASSERT(n <= MaxCount(b.countType));
				src = static_cast<const uint8_t*>(items);
				text[0] = ' ';
				Put(std::to_chars(text+1, text+32, n).ptr - (text+1), first);
				first = false;
			}
			for (size_t k = 0; k < n; ++k, src += typeSizes[b.type]) {
				const uint8_t* v(src);
				if (converts[p]) {
					converts[p](v, value, 1);
					v = value;
				}
				text[0] = ' ';
				Put(FormatValue(text+1, b.fileType, v), first);
				first = false;
			}
		}
		text[0] = '\n';
		Put(0, false);
	}
	return out.ok;
}

} // namespace


// a file property and the memory its values go to
struct PLY::Column {
	const Property* prop;
	const Binding* binding; // NULL: skip the property
	uint32_t size;          // bytes in the file of the scalar, or of the list count
	uint32_t itemSize;      // list: bytes in the file of an item
	ConvertFn convert;      // file to memory type, NULL: the bytes are copied
};


/*************/
/*  Reading  */
/*************/

bool PLY::Open(const std::string& fileName)
{
	ASSERT(elements.empty());
	file.open(fileName.c_str(), File::READ, File::OPEN);
	if (!file.isOpen())
		return false;
	fileLeft = file.getSize();
	if (fileLeft <= 0)
		return false;
	buffer.resize((size_t)std::min<size_f_t>(BUFFER_SIZE, fileLeft));
	return ParseHeader();
}

// make at least numBytes unread bytes available in the buffer; false if the file ends sooner
bool PLY::Fill(size_t numBytes)
{
	size_t unread(bufferEnd - bufferPos);
	if (unread >= numBytes)
		return true;
	if ((size_f_t)(numBytes - unread) > fileLeft)
		return false;
	// move the unread bytes to the front, then read as much as fits
	memmove(buffer.data(), buffer.data()+bufferPos, unread);
	bufferPos = 0;
	bufferEnd = unread;
	if (buffer.size() < numBytes)
		buffer.resize(std::max(numBytes, std::min(buffer.size()*2, unread + (size_t)fileLeft)));
	const size_t size((size_t)std::min<size_f_t>(buffer.size() - bufferEnd, fileLeft));
	if (!ReadChunked(file, buffer.data()+bufferEnd, size))
		return false;
	bufferEnd += size;
	fileLeft -= size;
	return true;
}

bool PLY::ReadLine(std::string& line)
{
	for (;;) {
		const uint8_t* const begin(buffer.data()+bufferPos);
		const uint8_t* const end(static_cast<const uint8_t*>(memchr(begin, '\n', bufferEnd-bufferPos)));
		if (end) {
			line.assign(reinterpret_cast<const char*>(begin), end-begin);
			if (!line.empty() && line.back() == '\r')
				line.pop_back();
			bufferPos = end+1 - buffer.data();
			return true;
		}
		if (bufferEnd-bufferPos >= MAX_HEADER_LINE || !Fill(bufferEnd-bufferPos+1))
			return false;
	}
}

bool PLY::ParseHeader()
{
	std::string line;
	if (!ReadLine(line) || line != "ply")
		return false;
	bool hasFormat(false);
	std::vector<std::string> words;
	for (;;) {
		if (!ReadLine(line))
			return false;
		words.clear();
		for (size_t i = 0; i < line.size(); ) {
			while (i < line.size() && IsSpace(line[i]))
				++i;
			const size_t start(i);
			while (i < line.size() && !IsSpace(line[i]))
				++i;
			if (i > start)
				words.emplace_back(line, start, i-start);
		}
		if (words.empty())
			continue;
		const std::string& keyword(words[0]);
		if (keyword == "format") {
			if (words.size() < 2)
				return false;
			if (words[1] == "ascii")
				fileType = ASCII;
			else if (words[1] == "binary_little_endian")
				fileType = BINARY_LE;
			else if (words[1] == "binary_big_endian")
				fileType = BINARY_BE;
			else
				return false;
			hasFormat = true;
		} else if (keyword == "comment") {
			// the text after the keyword and the blanks following it
			size_t start(line.find(keyword) + keyword.size());
			while (start < line.size() && (line[start] == ' ' || line[start] == '\t'))
				++start;
			comments.emplace_back(line, start);
		} else if (keyword == "element") {
			if (words.size() != 3 || words[2].find_first_not_of("0123456789") != std::string::npos)
				return false;
			elements.push_back(Element{words[1], (size_t)strtoull(words[2].c_str(), NULL, 10), {}});
		} else if (keyword == "property") {
			if (elements.empty())
				return false;
			Property prop;
			if (words.size() == 5 && words[1] == "list") {
				prop.countType = ParseType(words[2]);
				prop.type = ParseType(words[3]);
				prop.name = words[4];
				if (!IsInteger(prop.countType) || prop.type == NoType)
					return false;
			} else if (words.size() == 3) {
				prop.countType = NoType;
				prop.type = ParseType(words[1]);
				prop.name = words[2];
				if (prop.type == NoType)
					return false;
			} else {
				return false;
			}
			elements.back().props.emplace_back(std::move(prop));
		} else if (keyword == "end_header") {
			break;
		}
		// any other line, like obj_info, carries nothing we use
	}
	if (!hasFormat)
		return false;
	// reject element counts the file is too short to hold, so callers can size their arrays by them:
	// a binary record takes at least its scalars and list counts, an ascii value at least a character
	// and a separator
	const uint64_t bodySize((uint64_t)(bufferEnd-bufferPos) + (uint64_t)fileLeft);
	uint64_t minBodySize(0);
	for (const Element& elem: elements) {
		if (elem.count == 0)
			continue;
		if (elem.props.empty())
			return false;
		uint64_t minRecordSize(0);
		for (const Property& prop: elem.props)
			minRecordSize += fileType == ASCII ? 2 : typeSizes[prop.IsList() ? prop.countType : prop.type];
		if ((uint64_t)elem.count > (bodySize+1)/minRecordSize)
			return false;
		minBodySize += (uint64_t)elem.count*minRecordSize;
		if (minBodySize > bodySize+1)
			return false;
	}
	return true;
}

bool PLY::ReadElement(size_t idx, const Bindings& bindings)
{
	ASSERT(idx < elements.size() && idx >= nextElement); // elements are read in file order
	// skip the elements before it
	while (nextElement < idx)
		if (!ReadElement(nextElement, Bindings()))
			return false;
	++nextElement;
	const Element& elem(elements[idx]);
	// pair the file properties with the bindings
	std::vector<Column> columns(elem.props.size());
	for (size_t p = 0; p < elem.props.size(); ++p) {
		const Property& prop(elem.props[p]);
		columns[p] = Column{&prop, NULL, typeSizes[prop.IsList() ? prop.countType : prop.type], prop.IsList() ? typeSizes[prop.type] : 0, NULL};
	}
	for (const Binding& b: bindings) {
		Column* col(NULL);
		for (Column& c: columns)
			if (c.prop->name == b.name)
				col = &c;
		if (col == NULL || col->prop->IsList() != (b.countType != NoType)) {
			VERBOSE("error: PLY element '%s' has no %s property '%s'", elem.name.c_str(), b.countType != NoType ? "list" : "scalar", b.name);
			return false;
		}
		col->binding = &b;
		col->convert = Converter(col->prop->type, b.type, fileType == BINARY_BE);
	}
	// an empty element has no data, and its bindings may point nowhere
	if (elem.count == 0)
		return true;
	if (fileType == ASCII ? ReadAscii(elem, columns) : ReadBinary(elem, columns))
		return true;
	VERBOSE("error: invalid PLY element '%s'", elem.name.c_str());
	return false;
}

bool PLY::ReadBinary(const Element& elem, std::vector<Column>& columns)
{
	const bool swap(fileType == BINARY_BE);
	// the record splits into runs of fixed-size properties (scalars and lists bound to a fixed size),
	// stored at fixed offsets, separated by the lists whose size varies
	struct Op {
		uint32_t offset;     // in the run
		uint32_t bytes;      // copied bytes, when not converted
		uint32_t numItems;   // values per record
		uint32_t countOffset;// fixed-size list: offset of its count in the run
		DataType countType;  // fixed-size list: type of its count, else NoType
		const char* name;    // fixed-size list: its name, for the diagnostic
		uint8_t* dst;
		size_t stride;
		ConvertFn convert;
	};
	struct Step {
		uint32_t size;       // run: bytes
		std::vector<Op> ops; // run: the bound properties
		const Column* list;  // a list of variable size, or NULL for a run
	};
	std::vector<Step> steps;
	for (const Column& col: columns) {
		const Binding* const b(col.binding);
		if (col.prop->IsList() && (b == NULL || b->numItems == 0)) {
			steps.push_back(Step{0, {}, &col});
			continue;
		}
		if (steps.empty() || steps.back().list != NULL)
			steps.push_back(Step{0, {}, NULL});
		Step& run(steps.back());
		Op op;
		op.countOffset = run.size;
		if (col.prop->IsList()) {
			op.countType = col.prop->countType;
			op.name = col.prop->name.c_str();
			op.offset = run.size + col.size;
			op.numItems = b->numItems;
			op.bytes = b->numItems*col.itemSize;
		} else {
			op.countType = NoType;
			op.offset = run.size;
			op.numItems = 1;
			op.bytes = col.size;
		}
		run.size = op.offset + op.bytes;
		if (b == NULL)
			continue;
		op.dst = static_cast<uint8_t*>(b->data);
		op.stride = b->stride;
		op.convert = col.convert;
		// a plain copy of bytes adjacent to the previous plain copy, into adjacent memory, joins it
		if (!run.ops.empty()) {
			Op& prev(run.ops.back());
			if (op.convert == NULL && prev.convert == NULL && op.countType == NoType && prev.countType == NoType &&
				prev.offset + prev.bytes == op.offset && prev.dst + prev.bytes == op.dst && prev.stride == op.stride) {
				prev.bytes += op.bytes;
				continue;
			}
		}
		run.ops.push_back(op);
	}
	// store the bound properties of the run starting at src into record i
	const auto ApplyRun = [swap](const Step& run, const uint8_t* src, size_t i) {
		for (const Op& op: run.ops) {
			if (op.countType != NoType) {
				uint32_t count;
				if (!DecodeCount(src+op.countOffset, op.countType, swap, count))
					return false;
				if (count != op.numItems) {
					VERBOSE("error: PLY list '%s' has %u items, %u expected", op.name, count, op.numItems);
					return false;
				}
			}
			uint8_t* const dst(op.dst + i*op.stride);
			if (op.convert)
				op.convert(src+op.offset, dst, op.numItems);
			else
				memcpy(dst, src+op.offset, op.bytes);
		}
		return true;
	};

	if (steps.size() == 1 && steps[0].list == NULL) {
		// fixed-size records
		const Step& run(steps[0]);
		const size_t recordSize(run.size);
		const Op* const op(run.ops.size() == 1 ? &run.ops[0] : NULL);
		if (op && op->convert == NULL && op->countType == NoType && op->bytes == recordSize && op->stride == recordSize) {
			// the records are laid out as in memory: read them there directly
			const size_t size(elem.count*recordSize);
			const size_t buffered(std::min(size, bufferEnd-bufferPos));
			memcpy(op->dst, buffer.data()+bufferPos, buffered);
			bufferPos += buffered;
			if ((size_f_t)(size-buffered) > fileLeft || !ReadChunked(file, op->dst+buffered, size-buffered))
				return false;
			fileLeft -= size-buffered;
			return true;
		}
		for (size_t i = 0; i < elem.count; ) {
			if (!Fill(recordSize))
				return false;
			const uint8_t* record(buffer.data()+bufferPos);
			const size_t end(i + std::min(elem.count-i, (bufferEnd-bufferPos)/recordSize));
			for (; i < end; ++i, record += recordSize)
				if (!ApplyRun(run, record, i))
					return false;
			bufferPos = record - buffer.data();
		}
		return true;
	}

	// variable-size records
	std::vector<uint8_t> items;
	for (size_t i = 0; i < elem.count; ++i) {
		for (const Step& step: steps) {
			if (step.list == NULL) {
				if (!Fill(step.size) || !ApplyRun(step, buffer.data()+bufferPos, i))
					return false;
				bufferPos += step.size;
				continue;
			}
			const Column& col(*step.list);
			uint32_t count;
			if (!Fill(col.size) || !DecodeCount(buffer.data()+bufferPos, col.prop->countType, swap, count))
				return false;
			const size_t size(col.size + (size_t)count*col.itemSize);
			if (!Fill(size))
				return false;
			if (const Binding* const b = col.binding) {
				const uint8_t* src(buffer.data()+bufferPos+col.size);
				if (col.convert) {
					items.resize((size_t)count*typeSizes[b->type]);
					col.convert(src, items.data(), count);
					src = items.data();
				}
				b->setList(b->data, i, count, src);
			}
			bufferPos += size;
		}
	}
	return true;
}

// an ascii record is one line: skip the blanks before its next value; false at the end of the file
bool PLY::SkipBlanks()
{
	for (;;) {
		while (bufferPos < bufferEnd && IsBlank(buffer[bufferPos]))
			++bufferPos;
		if (bufferPos < bufferEnd)
			return true;
		if (!Fill(1))
			return false;
	}
}

// the next value of the record; false at the end of its line
bool PLY::NextToken(const char*& token, size_t& len)
{
	if (!SkipBlanks() || buffer[bufferPos] == '\n')
		return false;
	for (;;) {
		size_t end(bufferPos);
		while (end < bufferEnd && !IsSpace(buffer[end]))
			++end;
		if (end < bufferEnd || fileLeft == 0) {
			token = reinterpret_cast<const char*>(buffer.data()+bufferPos);
			len = end-bufferPos;
			bufferPos = end;
			return true;
		}
		// the token may continue past the buffer
		if (bufferEnd-bufferPos >= MAX_TOKEN || !Fill(bufferEnd-bufferPos+1))
			return false;
	}
}

// the record is complete: only blanks may follow it on its line (the last line may lack its newline)
bool PLY::EndLine()
{
	return !SkipBlanks() || buffer[bufferPos++] == '\n';
}

bool PLY::ReadAscii(const Element& elem, std::vector<Column>& columns)
{
	// parse the next value as type t into v
	const auto ParseValue = [this](DataType t, uint8_t* v) {
		const char* token;
		size_t len;
		char text[MAX_TOKEN];
		if (!NextToken(token, len) || len >= sizeof(text))
			return false;
		memcpy(text, token, len);
		text[len] = '\0';
		char* end;
		if (t == Float32) {
			const float x(strtof(text, &end));
			memcpy(v, &x, sizeof(float));
		} else if (t == Float64) {
			const double x(strtod(text, &end));
			memcpy(v, &x, sizeof(double));
		} else {
			long long x(strtoll(text, &end, 10));
			if (*end != '\0')
				x = static_cast<long long>(strtod(text, &end)); // an integer written as a real
			Dispatch(t, [v, x](auto y) { y = static_cast<decltype(y)>(x); memcpy(v, &y, sizeof(y)); });
		}
		return end == text+len;
	};
	uint8_t value[8];
	std::vector<uint8_t> values, items;
	for (size_t i = 0; i < elem.count; ++i) {
		for (const Column& col: columns) {
			const Binding* const b(col.binding);
			if (!col.prop->IsList()) {
				if (!ParseValue(col.prop->type, value))
					return false;
				if (b) {
					uint8_t* const dst(static_cast<uint8_t*>(b->data) + i*b->stride);
					if (col.convert)
						col.convert(value, dst, 1);
					else
						memcpy(dst, value, col.size);
				}
				continue;
			}
			uint32_t count;
			if (!ParseValue(col.prop->countType, value) || !DecodeCount(value, col.prop->countType, false, count))
				return false;
			// each item takes at least a character and a separator
			if ((size_f_t)count > ((size_f_t)(bufferEnd-bufferPos) + fileLeft + 1)/2)
				return false;
			values.resize((size_t)count*col.itemSize);
			for (uint32_t k = 0; k < count; ++k)
				if (!ParseValue(col.prop->type, values.data()+k*col.itemSize))
					return false;
			if (b == NULL)
				continue;
			const uint8_t* src(values.data());
			if (col.convert) {
				items.resize((size_t)count*typeSizes[b->type]);
				col.convert(src, items.data(), count);
				src = items.data();
			}
			if (b->numItems) {
				if (count != b->numItems) {
					VERBOSE("error: PLY list '%s' has %u items, %u expected", b->name, count, b->numItems);
					return false;
				}
				memcpy(static_cast<uint8_t*>(b->data) + i*b->stride, src, (size_t)count*typeSizes[b->type]);
			} else {
				b->setList(b->data, i, count, src);
			}
		}
		if (!EndLine())
			return false;
	}
	return true;
}


/*************/
/*  Writing  */
/*************/

void PLY::AddElement(const char* name, size_t count, Bindings bindings)
{
	ASSERT(count == 0 || !bindings.empty());
	outElements.push_back(OutElement{name, count, std::move(bindings)});
}

bool PLY::Save(const std::string& fileName, FileType type, bool legacyTypeNames) const
{
	ASSERT(type == ASCII || type == BINARY_LE);
	Writer out(fileName);
	if (!out.ok)
		return false;
	// header
	const char* const* const names(legacyTypeNames ? oldTypeNames : typeNames);
	std::string header(type == ASCII ? "ply\nformat ascii 1.0\n" : "ply\nformat binary_little_endian 1.0\n");
	for (const std::string& comment: comments)
		header += "comment " + comment + "\n";
	for (const OutElement& elem: outElements) {
		header += "element " + elem.name + " " + std::to_string(elem.count) + "\n";
		for (const Binding& b: elem.bindings) {
			if (b.countType == NoType)
				header += std::string("property ") + names[b.fileType] + " " + b.name + "\n";
			else
				header += std::string("property list ") + names[b.countType] + " " + names[b.fileType] + " " + b.name + "\n";
		}
	}
	header += "end_header\n";
	out.Write(header.data(), header.size());
	// body
	for (const OutElement& elem: outElements)
		if (!(type == ASCII ? WriteAscii(out, elem.count, elem.bindings) : WriteBinary(out, elem.count, elem.bindings)))
			return false;
	return out.Flush();
}
/*----------------------------------------------------------------*/
