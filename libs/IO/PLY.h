/*

PLY polygon files parser.
(originally by Greg Turk, heavily modified by cDc@seacave)

A PLY file contains a single polygonal _object_.

An object is composed of lists of _elements_.  Typical elements are
vertices, faces, edges and materials.

Each type of element for a given object has one or more _properties_
associated with the element type.  For instance, a vertex element may
have as properties three floating-point values x,y,z and three unsigned
chars for red, green and blue.

*/

#ifndef __PLY_H__
#define __PLY_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////

#define PLY_OKAY    0           /* ply routine worked okay */
#define PLY_ERROR  -1           /* error in ply routine */


// S T R U C T S ///////////////////////////////////////////////////

class IO_API PLY
{
public:
	// scalar data types supported by PLY format
	enum FileType {
		ASCII     = 1, // ascii PLY file
		BINARY_BE = 2, // binary PLY file, big endian
		BINARY_LE = 3, // binary PLY file, little endian
	};
	enum DataType {
		StartType = 0,
		Int8      = 1,
		Int16     = 2,
		Int32     = 3,
		Uint8     = 4,
		Uint16    = 5,
		Uint32    = 6,
		Float32   = 7,
		Float64   = 8,
		EndType   = 9,
	};
	enum ListType {
		SCALAR    = 0,
		LIST      = 1,
		STRING    = 2,
	};
	enum RuleType {
		AVERAGE_RULE  = 1,
		MAJORITY_RULE = 2,
		MINIMUM_RULE  = 3,
		MAXIMUM_RULE  = 4,
		SAME_RULE     = 5,
		RANDOM_RULE   = 6,
	};

	// description of a property
	struct PlyProperty {
		std::string name;             /* property name */
		int external_type;            /* file's data type */
		int internal_type;            /* program's data type */
		int offset;                   /* offset bytes of prop in a struct */

		int is_list;                  /* 0 = scalar, 1 = list, 2 = char string */
		int count_external;           /* file's count type */
		int count_internal;           /* program's count type */
		int count_offset;             /* offset byte for list count */
	};

	// description of an element
	struct PlyElement {
		std::string name;             /* element name */
		int num;                      /* number of elements in this object */
		int size;                     /* size of element (bytes) or -1 if variable */
		std::vector<PlyProperty*> props;/* list of properties in the file */
		std::vector<char> store_prop; /* flags: property wanted by user? */
		int other_offset;             /* offset to un-asked-for props, or -1 if none*/
		int other_size;               /* size of other_props structure */
	};

	// describes other properties in an element
	struct PlyOtherProp {
		std::string name;             /* element name */
		int size;                     /* size of other_props */
		std::vector<PlyProperty*> props;/* list of properties in other_props */
	};

	// for storing other_props for an other element
	struct OtherData {
		void *other_props;
	};

	// data for one "other" element
	struct OtherElem {
		std::string elem_name;       /* names of other elements */
		int elem_count;              /* count of instances of each element */
		OtherData **other_data;      /* actual property data for the elements */
		PlyOtherProp *other_props;   /* description of the property data */
	};

	// "other" elements, not interpreted by user
	struct PlyOtherElems {
		std::vector<OtherElem> other_list;/* list of data for other elements */
	};

	// rules for combining "other" properties
	struct PlyPropRules {
		PlyElement *elem;      /* element whose rules we are making */
		int *rule_list;        /* types of rules (AVERAGE_PLY, MAJORITY_PLY, etc.) */
		uint32_t max_props;    /* maximum number of properties we have room for now */
		std::vector<void*> props;/* list of properties we're combining */
		std::vector<float> weights;/* list of weights of the properties */
	};

	struct PlyRuleList {
		LPCSTR name;                 /* name of the rule */
		char *element;               /* name of element that rule applies to */
		char *property;              /* name of property that rule applies to */
		struct PlyRuleList *next;    /* pointer for linked list of rules */
	};

	// property propagation rules
	struct RuleName {
		int code;
		std::string name;
	};

protected:
	struct ValueType {
		union {
			int8_t i8;
			int16_t i16;
			int32_t i32;
			uint8_t u8;
			uint16_t u16;
			uint32_t u32;
			float f;
			double d;
		};
	};

public:
	PLY();
	~PLY();

	bool read(LPCSTR);
	bool read(SEACAVE::ISTREAM*);
	bool write(LPCSTR, int, LPCSTR*, int, size_t = 0);
	bool write(SEACAVE::OSTREAM*, int, LPCSTR*, int, size_t = 0);
	void flush();
	void release();

	void get_info(float *, int *);

	void append_comment(const char *);
	void append_obj_info(const char *);
	void copy_comments(const PLY&);
	void copy_obj_info(const PLY&);
	std::vector<std::string>& get_comments();
	std::vector<std::string>& get_obj_info();

	void describe_property(const char *, const PlyProperty&);
	void describe_property(const char *, int nprops, const PlyProperty*);
	void get_property(const char *, PlyProperty *);
	void get_element(void*);

	PlyOtherElems *get_other_element();

	int get_element_list(std::vector<std::string>&) const;
	void setup_property(const PlyProperty&);
	LPCSTR setup_element_read(int, int*);
	PlyOtherProp *get_other_properties(int);

	void element_count(const char *, int);
	int get_current_element_count() const { return which_elem->num; }
	void describe_element(char *, int);
	void describe_property(const PlyProperty&);
	void describe_other_properties(PlyOtherProp *, int);
	void describe_other_elements( PlyOtherElems *);
	void get_element_setup(const char *, int, PlyProperty *);
	int get_element_description(const char *, std::vector<PlyProperty*>&) const;
	void element_layout(const char *, int, int, PlyProperty *);

	bool header_complete();
	void put_element_setup(const char *);
	void put_element(const void*);
	void put_other_elements();

	PlyPropRules *init_rule(const char *);
	void modify_rule(PlyPropRules *, const char *, int);
	void start_props(PlyPropRules *);
	void weight_props(float, void *);
	void *get_new_props();
	void set_prop_rules(PlyRuleList *);
	PlyRuleList *append_prop_rule(PlyRuleList *, const char *, const char *);

	/* find an element in a ply's list */
	PlyElement* find_element(const char *) const;
	/* find a property in an element's list */
	int find_property(PlyElement *, const char *) const;

	static inline bool equal_strings(const char* s1, const char* s2) { return _tcscmp(s1, s2) == 0; }

protected:
	/* write to a file the word describing a PLY file data type */
	void write_scalar_type(SEACAVE::OSTREAM*, int);

	/* read a line from a file and break it up into separate words */
	typedef SEACAVE::TokenInputStream<false> STRISTREAM;
	char **get_words(STRISTREAM&, int *, char **);

	/* write an item to a file */
	void write_binary_item(const ValueType&, int, int);
	void write_ascii_item(const ValueType&, int, int);

	/* return the value of a stored item */
	void get_stored_item(void*, int, ValueType&);

	/* get binary or ascii item and store it according to ptr and type */
	void get_binary_item(SEACAVE::ISTREAM*, int, ValueType&);
	void get_ascii_item(const char*, int, ValueType&);

	/* store a value into where a pointer and a type specify */
	void store_item(void*, int, const ValueType&, int);

	/* add information to a PLY file descriptor */
	void add_element(const char **, int);
	void add_property(const char **, int);
	void add_comment(const char *);
	void add_obj_info(const char *);

	/* get a bunch of elements from a file */
	void ascii_get_element(uint8_t*);
	void binary_get_element(uint8_t*);

	void setup_other_props(PlyElement *);
	PlyOtherProp *get_other_properties(PlyElement *, int);
	PlyOtherProp *get_other_properties(const char *, int);

	int matches_rule_name(const char *);
	int get_prop_type(const char *);

	/* copy a property */
	static void copy_property(PlyProperty&, const PlyProperty&);

	/* return the value as T stored in val as type */
	template <typename T>
	static inline T ValueType2Type(const ValueType& val, int type) {
		switch (type) {
		case Int8:
			return (T)val.i8;
		case Int16:
			return (T)val.i16;
		case Int32:
			return (T)val.i32;
		case Uint8:
			return (T)val.u8;
		case Uint16:
			return (T)val.u16;
		case Uint32:
			return (T)val.u32;
		case Float32:
			return (T)val.f;
		case Float64:
			return (T)val.d;
		}
		ASSERT("error: bad type" == NULL);
		return T(0);
	}

public:
	// description of PLY file
	std::string filename;          /* file name */
	SEACAVE::MemFile* mfp;         /* mem file pointer */
	SEACAVE::OSTREAM* f;           /* output file pointer */
	union {
		SEACAVE::ISTREAM* istream; /* input file pointer */
		SEACAVE::OSTREAM* ostream; /* output file pointer */
	};
	int file_type;                 /* ascii or binary */
	float version;                 /* version number of file */
	std::vector<PlyElement*> elems;/* list of elements */
	std::vector<std::string> comments;/* list of comments */
	std::vector<std::string> obj_info;/* list of object info items */
	PlyElement *which_elem;        /* element we're currently reading or writing */
	PlyOtherElems *other_elems;    /* "other" elements from a PLY file */
	PlyPropRules *current_rules;   /* current propagation rules */
	PlyRuleList *rule_list;        /* rule list from user */
	std::vector<double> vals;      /* rule list from user */

protected:
	static const char *type_names[9]; // names of scalar types
	static const char *old_type_names[9]; // old names of types for backward compatibility
	static const int ply_type_size[9];
	static const RuleName rule_name_list[7];
};

#endif // __PLY_H__
