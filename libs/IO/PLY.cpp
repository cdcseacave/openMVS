////////////////////////////////////////////////////////////////////
// PLY.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "PLY.h"

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////

#define NO_OTHER_PROPS  -1

#define DONT_STORE_PROP  0
#define STORE_PROP       1

#define OTHER_PROP       0
#define NAMED_PROP       1

#define abort_ply(...)   { VERBOSE(__VA_ARGS__); exit(-1); }


// S T R U C T S ///////////////////////////////////////////////////

const char* const PLY::type_names[] = {
	"invalid", "int8", "int16", "int32", "uint8", "uint16", "uint32", "float32", "float64",
};

const char* const PLY::old_type_names[] = {
	"invalid", "char", "short", "int", "uchar", "ushort", "uint", "float", "double",
};

const int PLY::ply_type_size[] = {
	0, 1, 2, 4, 1, 2, 4, 4, 8
};

const PLY::RuleName PLY::rule_name_list[] = {
	{AVERAGE_RULE, "avg"},
	{RANDOM_RULE, "rnd"},
	{MINIMUM_RULE, "max"},
	{MAXIMUM_RULE, "min"},
	{MAJORITY_RULE, "major"},
	{SAME_RULE, "same"},
	{-1, "end_marker"}
};



/******************/
/*  Construction  */
/******************/


/******************************************************************************
Init PLY data as empty.

Entry:

Exit:
******************************************************************************/

PLY::PLY()
	:
	mfp(NULL), f(NULL), ostream(NULL),
	which_elem(NULL), other_elems(NULL), current_rules(NULL), rule_list(NULL),
	write_type_names(type_names)
{
}

PLY::~PLY()
{
	release();
}


/******************************************************************************
Free the memory used by a PLY file.
******************************************************************************/

void PLY::release()
{
	flush();
	if (mfp) {
		delete mfp;
		mfp = NULL;
	}
	f = NULL;
	if (!filename.empty()) {
		filename.clear();
		delete ostream;
	}
	ostream = NULL;
	if (!elems.empty()) {
		for (size_t i=0; i<elems.size(); ++i) {
			PlyElement* elem = elems[i];
			if (!elem->props.empty()) {
				for (size_t j=0; j<elem->props.size(); ++j)
					delete elem->props[j];
				elem->props.clear();
				elem->store_prop.clear();
			}
			delete elem;
		}
		elems.clear();
	}
	if (other_elems != NULL) {
		for (size_t i=0; i<other_elems->other_list.size(); ++i) {
			OtherElem& elem = other_elems->other_list[i];
			delete[] elem.other_data;
			delete elem.other_props;
		}
		delete other_elems;
	}
	comments.clear();
	obj_info.clear();
	vals.clear();
}



/*************/
/*  Writing  */
/*************/


/******************************************************************************
Given a file pointer, get ready to write PLY data to the file.

Entry:
f          - the given file pointer
nelems     - number of elements in object
elem_names - list of element names
file_type  - file type, either ascii or binary

Exit:
returns a pointer to a PlyFile, used to refer to this file, or NULL if error
******************************************************************************/

bool PLY::write(LPCSTR filename, int nelems, LPCSTR* elem_names, int file_type, size_t bufferSize)
{
	this->filename = filename;
	return write(ostream, nelems, elem_names, file_type, bufferSize);
}

bool PLY::write(OSTREAM* f, int nelems, LPCSTR* elem_names, int file_type, size_t bufferSize)
{
	/* create a record for this object */
	this->file_type = file_type;
	this->version = 1.0;
	this->ostream = f;
	this->other_elems = NULL;

	/* init buffer if requested */
	if (bufferSize > 0) {
		mfp = new MemFile(bufferSize);
		this->f = mfp;
	} else {
		this->f = f;
	}

	/* tuck aside the names of the elements */
	this->elems.resize(nelems);
	for (int i = 0; i < nelems; ++i) {
		PlyElement* elem = new PlyElement;
		elem->name = elem_names[i];
		elem->num = 0;
		this->elems[i] = elem;
	}

	return true;
}


/******************************************************************************
Describe an element, including its properties and how many will be written
to the file.

Entry:
elem_name - name of element that information is being specified about
nelems    - number of elements of this type to be written
nprops    - number of properties contained in the element
prop_list - list of properties
******************************************************************************/

void PLY::element_layout(
	const char *elem_name,
	int nelems,
	int nprops,
	PlyProperty *prop_list
	)
{
	/* look for appropriate element */
	PlyElement *elem = find_element(elem_name);
	if (elem == NULL)
		abort_ply("error: element_layout: can't find element '%s'", elem_name);
	elem->num = nelems;

	/* copy the list of properties */
	elem->props.resize(nprops);
	elem->store_prop.resize(nprops);

	for (int i = 0; i < nprops; ++i) {
		PlyProperty *prop = new PlyProperty;
		elem->props[i] = prop;
		elem->store_prop[i] = NAMED_PROP;
		copy_property(*prop, prop_list[i]);
	}
}


/******************************************************************************
Describe a property of an element.

Entry:
elem_name - name of element that information is being specified about
prop      - the new property
******************************************************************************/

void PLY::describe_property(const char *elem_name, const PlyProperty& prop)
{
	/* look for appropriate element */
	put_element_setup(elem_name);

	/* describe property */
	describe_property(prop);
}

void PLY::describe_property(const char *elem_name, int nprops, const PlyProperty* props)
{
	/* look for appropriate element */
	put_element_setup(elem_name);

	/* describe properties */
	for (int i=0; i<nprops; ++i)
		describe_property(props[i]);
}


/******************************************************************************
State how many of a given element will be written.

Entry:
elem_name - name of element that information is being specified about
nelems    - number of elements of this type to be written
******************************************************************************/

void PLY::element_count(const char *elem_name, int nelems)
{
	/* look for appropriate element */
	PlyElement *elem = find_element(elem_name);
	if (elem == NULL)
		abort_ply("error: element_count: can't find element '%s'", elem_name);
	elem->num = nelems;
}


/******************************************************************************
Signal that we've described everything a PLY file's header and that the
header should be written to the file.

If buffered mode used, call this function at the end, when everything
was written already.
******************************************************************************/

bool PLY::header_complete()
{
	if (!filename.empty()) {
		ostream = new File(filename.c_str(), File::WRITE, File::CREATE | File::TRUNCATE);
		if (!((File*)ostream)->isOpen())
			return false;
		ostream = new BufferedOutputStream<true>(ostream, FILE_WRITE_MINBUF_SIZE);
	}

	ostream->print("ply\n");

	switch (this->file_type) {
	case ASCII:
		ostream->print("format ascii 1.0\n");
		break;
	case BINARY_BE:
		ostream->print("format binary_big_endian 1.0\n");
		break;
	case BINARY_LE:
		ostream->print("format binary_little_endian 1.0\n");
		break;
	default:
		abort_ply("error: ply_header_complete: bad file type = %d\n", this->file_type);
	}

	/* write out the comments */
	for (size_t i = 0; i < this->comments.size(); ++i)
		ostream->print("comment %s\n", this->comments[i].c_str());

	/* write out object information */
	for (size_t i = 0; i < this->obj_info.size(); ++i)
		ostream->print("obj_info %s\n", this->obj_info[i].c_str());

	/* write out information about each element */
	for (size_t i = 0; i < this->elems.size(); ++i) {
		PlyElement *elem = this->elems[i];
		ostream->print("element %s %d\n", elem->name.c_str(), elem->num);

		/* write out each property */
		for (size_t j = 0; j < elem->props.size(); ++j) {
			PlyProperty *prop = elem->props[j];
			if (prop->is_list == LIST) {
				ostream->print("property list ");
				write_scalar_type(ostream, prop->count_external);
				ostream->print(" ");
				write_scalar_type(ostream, prop->external_type);
				ostream->print(" %s\n", prop->name.c_str());
			}
			else if (prop->is_list == STRING) {
				ostream->print("property string");
				ostream->print(" %s\n", prop->name.c_str());
			}
			else {
				ostream->print("property ");
				write_scalar_type(ostream, prop->external_type);
				ostream->print(" %s\n", prop->name.c_str());
			}
		}
	}

	ostream->print("end_header\n");

	/* write the body also if fully buffered */
	if (mfp && mfp->getSize() > 0) {
		ostream->write(mfp->getBuffer(), mfp->getSize());
		delete mfp; mfp = NULL;
	}

	return true;
}


/******************************************************************************
Specify which elements are going to be written.  This should be called
before a call to the routine ply_put_element().

Entry:
elem_name - name of element we're talking about
******************************************************************************/

void PLY::put_element_setup(const char *elem_name)
{
	PlyElement *elem = find_element(elem_name);
	if (elem == NULL)
		abort_ply("error: put_element_setup: can't find element '%s'", elem_name);
	this->which_elem = elem;
}


/******************************************************************************
Write an element to the file.  This routine assumes that we're
writing the type of element specified in the last call to the routine
put_element_setup().

Entry:
elem_ptr - pointer to the element
******************************************************************************/

void PLY::put_element(const void* elem_ptr)
{
	char *item;
	char *elem_data;
	char **item_ptr;
	ValueType val;
	char **other_ptr;

	PlyElement *elem = this->which_elem;
	elem_data = (char *)elem_ptr;
	other_ptr = (char **)(((char *)elem_ptr) + elem->other_offset);

	/* write out either to an ascii or binary file */
	if (this->file_type == ASCII) {

		/* write an ascii file */

		/* write out each property of the element */
		for (size_t j = 0; j < elem->props.size(); ++j) {

			PlyProperty *prop = elem->props[j];

			if (elem->store_prop[j] == OTHER_PROP)
				elem_data = *other_ptr;
			else
				elem_data = (char *)elem_ptr;

			switch (prop->is_list) {
			case SCALAR: {  /* scalar */
				item = elem_data + prop->offset;
				get_stored_item((void*)item, prop->internal_type, val);
				write_ascii_item(val, prop->internal_type, prop->external_type);
				break;
			}
			case LIST: {   /* list */
				item = elem_data + prop->count_offset;
				get_stored_item((void*)item, prop->count_internal, val);
				write_ascii_item(val, prop->count_internal, prop->count_external);
				const int list_count(ValueType2Type<int>(val, prop->count_external));
				item_ptr = (char **)(elem_data + prop->offset);
				item = item_ptr[0];
				const int item_size = ply_type_size[prop->internal_type];
				for (int k = 0; k < list_count; k++) {
					get_stored_item((void*)item, prop->internal_type, val);
					write_ascii_item(val, prop->internal_type, prop->external_type);
					item += item_size;
				}
				break;
			}
			case STRING: {  /* string */
				item = elem_data + prop->offset;
				char **str = (char **)item;
				f->print("\"%s\"", *str);
				break;
			}
			default:
				abort_ply("error: put_element: bad type = %d", prop->is_list);
			}
		}

		f->print("\n");
	}
	else {

		/* write a binary file */

		/* write out each property of the element */
		for (size_t j = 0; j < elem->props.size(); ++j) {
			PlyProperty *prop = elem->props[j];
			if (elem->store_prop[j] == OTHER_PROP)
				elem_data = *other_ptr;
			else
				elem_data = (char *) elem_ptr;
			switch (prop->is_list) {
			case SCALAR: {  /* scalar */
				item = elem_data + prop->offset;
				get_stored_item((void*)item, prop->internal_type, val);
				write_binary_item(val, prop->internal_type, prop->external_type);
				break;
			}
			case LIST: {    /* list */
				item = elem_data + prop->count_offset;
				int item_size = ply_type_size[prop->count_internal];
				get_stored_item((void*)item, prop->count_internal, val);
				write_binary_item(val, prop->count_internal, prop->count_external);
				const int list_count(ValueType2Type<int>(val, prop->count_external));
				item_ptr = (char **)(elem_data + prop->offset);
				item = item_ptr[0];
				item_size = ply_type_size[prop->internal_type];
				for (int k = 0; k < list_count; k++) {
					get_stored_item((void*)item, prop->internal_type, val);
					write_binary_item(val, prop->internal_type, prop->external_type);
					item += item_size;
				}
				break;
			}
			case STRING: {  /* string */
				item = elem_data + prop->offset;
				char **str = (char **) item;

				/* write the length */
				const int len = (int)strlen(*str) + 1;
				f->write(&len, sizeof(int));

				/* write the string, including the null character */
				f->write(*str, len);
				break;
			}
			default:
				abort_ply("error: put_element: bad type = %d", prop->is_list);
			}
		}
	}

	/* if buffered mode, count element items */
	if (mfp) {
		if (ostream != NULL) {
			if (mfp->getSizeBuffer()-mfp->getSize() < 256) {
				ostream->write(mfp->getBuffer(), mfp->getSize());
				mfp->setSize(0);
			}
		} else {
			elem->num++;
		}
	}
}



/*************/
/*  Reading  */
/*************/


/******************************************************************************
Given a file pointer, get ready to read PLY data from the file.

Entry:
f          - the given file pointer

Exit:
nelems     - number of elements in object
elem_names - list of element names
returns a pointer to a PlyFile, used to refer to this file, or NULL if error
******************************************************************************/

bool PLY::read(LPCSTR filename)
{
	this->filename = filename;
	istream = new File(filename, File::READ, File::OPEN);
	if (!((File*)istream)->isOpen())
		return false;
	return read(new BufferedInputStream<true>(istream, FILE_READ_MINBUF_SIZE));
}

bool PLY::read(ISTREAM* f)
{
	/* create record for this object */
	ASSERT(elems.empty());
	this->istream = f;
	this->other_elems = NULL;
	this->rule_list = NULL;

	/* read and parse the file's header */
	int nwords;
	char *orig_line;
	STRISTREAM sfp(istream);
	char **words = get_words(sfp, &nwords, &orig_line);
	if (words == NULL)
		return false;
	if (!equal_strings (words[0], "ply")) {
		free(words);
		return false;
	}
	free(words);

	/* parse words */
	while ((words = get_words(sfp, &nwords, &orig_line)) != NULL) {
		if (equal_strings(words[0], "format")) {
			if (nwords != 3)
				return false;
			if (equal_strings(words[1], "ascii"))
				this->file_type = ASCII;
			else if (equal_strings(words[1], "binary_big_endian"))
				this->file_type = BINARY_BE;
			else if (equal_strings(words[1], "binary_little_endian"))
				this->file_type = BINARY_LE;
			else
				return false;
			this->version = (float)atof(words[2]);
		}
		else if (equal_strings(words[0], "element"))
			add_element((const char**)words, nwords);
		else if (equal_strings(words[0], "property"))
			add_property((const char**)words, nwords);
		else if (equal_strings(words[0], "comment"))
			add_comment(orig_line);
		else if (equal_strings(words[0], "obj_info"))
			add_obj_info(orig_line);
		else if (equal_strings(words[0], "end_header")) {
			free(words);
			break;
		}
		free(words);
	}
	sfp.emptyBuffer();

	/* create tags for each property of each element, to be used */
	/* later to say whether or not to store each property for the user */
	for (size_t i = 0; i < this->elems.size(); ++i) {
		PlyElement *elem = this->elems[i];
		elem->store_prop.resize(elem->props.size());
		for (size_t j = 0; j < elem->props.size(); ++j)
			elem->store_prop[j] = DONT_STORE_PROP;
		elem->other_offset = NO_OTHER_PROPS; /* no "other" props by default */
	}

	return true;
}


/******************************************************************************
Get information about a particular element.

Entry:
elem_name - name of element to get information about

Exit:
props     - the list of properties returned
returns number of elements of this type in the file
******************************************************************************/

int PLY::get_element_description(const char *elem_name, std::vector<PlyProperty*>& prop_list) const
{
	/* find information about the element */
	PlyElement *elem = find_element(elem_name);
	if (elem == NULL)
		return 0;

	/* make a copy of the element's property list */
	prop_list.resize(elem->props.size());
	for (size_t i = 0; i < elem->props.size(); ++i) {
		PlyProperty *prop = new PlyProperty;
		copy_property(*prop, *elem->props[i]);
		prop_list[i] = prop;
	}

	return elem->num;
}


/******************************************************************************
Specify which properties of an element are to be returned.  This should be
called before a call to the routine get_element().

Entry:
elem_name - which element we're talking about
nprops    - number of properties
prop_list - list of properties
******************************************************************************/

void PLY::get_element_setup(
	const char *elem_name,
	int nprops,
	PlyProperty *prop_list
	)
{
	/* find information about the element */
	PlyElement *elem = find_element(elem_name);
	this->which_elem = elem;

	/* deposit the property information into the element's description */
	for (int i = 0; i < nprops; ++i) {
		/* look for actual property */
		int index = find_property(elem, prop_list[i].name.c_str());
		if (index == -1) {
			DEBUG("warning: Can't find property '%s' in element '%s'", prop_list[i].name.c_str(), elem_name);
			continue;
		}

		/* store its description */
		PlyProperty *prop = elem->props[index];
		prop->internal_type = prop_list[i].internal_type;
		prop->offset = prop_list[i].offset;
		prop->count_internal = prop_list[i].count_internal;
		prop->count_offset = prop_list[i].count_offset;

		/* specify that the user wants this property */
		elem->store_prop[index] = STORE_PROP;
	}
}


/******************************************************************************
Specify a property of an element that is to be returned.  This should be
called (usually multiple times) before a call to the routine get_element().
This routine should be used in preference to the less flexible old routine
called ply_get_element_setup().

Entry:
elem_name - which element we're talking about
prop      - property to add to those that will be returned
******************************************************************************/

void PLY::get_property(
	const char *elem_name,
	PlyProperty *prop
	)
{
	/* find information about the element */
	PlyElement *elem = find_element(elem_name);
	this->which_elem = elem;

	/* deposit the property information into the element's description */
	int index = find_property(elem, prop->name.c_str());
	if (index == -1) {
		DEBUG("warning:  Can't find property '%s' in element '%s'", prop->name.c_str(), elem_name);
		return;
	}
	PlyProperty *prop_ptr = elem->props[index];
	prop_ptr->internal_type  = prop->internal_type;
	prop_ptr->offset         = prop->offset;
	prop_ptr->count_internal = prop->count_internal;
	prop_ptr->count_offset   = prop->count_offset;

	/* specify that the user wants this property */
	elem->store_prop[index] = STORE_PROP;
}


/******************************************************************************
Read one element from the file.  This routine assumes that we're reading
the type of element specified in the last call to the routine
ply_get_element_setup().

Entry:
elem_ptr - pointer to location where the element information should be put
******************************************************************************/

void PLY::get_element(void* elem_ptr)
{
	if (this->file_type == ASCII)
		ascii_get_element((uint8_t*)elem_ptr);
	else
		binary_get_element((uint8_t*)elem_ptr);
}


/******************************************************************************
Extract the comments from the header information of a PLY file.

Entry:

Exit:
returns the list of comments
******************************************************************************/

std::vector<std::string>& PLY::get_comments()
{
	return (this->comments);
}


/******************************************************************************
Extract the object information (arbitrary text) from the header information
of a PLY file.

Entry:

Exit:
returns the list of object info lines
******************************************************************************/

std::vector<std::string>& PLY::get_obj_info()
{
	return (this->obj_info);
}


/******************************************************************************
Make ready for "other" properties of an element-- those properties that
the user has not explicitly asked for, but that are to be stashed away
in a special structure to be carried along with the element's other
information.

Entry:
elem    - element for which we want to save away other properties
******************************************************************************/

void PLY::setup_other_props(PlyElement *elem)
{
	int size = 0;

	/* Examine each property in decreasing order of size. */
	/* We do this so that all data types will be aligned by */
	/* word, half-word, or whatever within the structure. */
	for (int type_size = 8; type_size > 0; type_size /= 2) {

		/* add up the space taken by each property, and save this information */
		/* away in the property descriptor */
		for (size_t i = 0; i < elem->props.size(); ++i) {

			/* don't bother with properties we've been asked to store explicitly */
			if (elem->store_prop[i])
				continue;

			PlyProperty *prop = elem->props[i];

			/* internal types will be same as external */
			prop->internal_type = prop->external_type;
			prop->count_internal = prop->count_external;

			/* list case */
			if (prop->is_list == LIST) {

				/* pointer to list */
				if (type_size == sizeof (void *)) {
					prop->offset = size;
					size += sizeof (void *);    /* always use size of a pointer here */
				}

				/* count of number of list elements */
				if (type_size == ply_type_size[prop->count_external]) {
					prop->count_offset = size;
					size += ply_type_size[prop->count_external];
				}
			}
			/* string */
			else if (prop->is_list == STRING) {
				/* pointer to string */
				if (type_size == sizeof (char *)) {
					prop->offset = size;
					size += sizeof (char *);
				}
			}
			/* scalar */
			else if (type_size == ply_type_size[prop->external_type]) {
				prop->offset = size;
				size += ply_type_size[prop->external_type];
			}
		}

	}

	/* save the size for the other_props structure */
	elem->other_size = size;
}


/******************************************************************************
Specify that we want the "other" properties of an element to be tucked
away within the user's structure.

Entry:
elem    - the element that we want to store other_props in
offset  - offset to where other_props will be stored inside user's structure

Exit:
returns pointer to structure containing description of other_props
******************************************************************************/

PLY::PlyOtherProp* PLY::get_other_properties(PlyElement *elem, int offset)
{
	/* remember that this is the "current" element */
	this->which_elem = elem;

	/* save the offset to where to store the other_props */
	elem->other_offset = offset;

	/* place the appropriate pointers, etc. in the element's property list */
	setup_other_props(elem);

	/* create structure for describing other_props */
	PlyOtherProp *other = new PlyOtherProp;
	other->name = elem->name;
	#if 0
	if (elem->other_offset == NO_OTHER_PROPS) {
		other->size = 0;
		other->props = NULL;
		other->nprops = 0;
		return (other);
	}
	#endif
	other->size = elem->other_size;
	other->props.reserve(elem->props.size());

	/* save descriptions of each "other" property */
	for (size_t i = 0; i < elem->props.size(); ++i) {
		if (elem->store_prop[i])
			continue;
		PlyProperty *prop = new PlyProperty;
		copy_property(*prop, *elem->props[i]);
		other->props.push_back(prop);
	}

	/* set other_offset pointer appropriately if there are NO other properties */
	if (other->props.empty())
		elem->other_offset = NO_OTHER_PROPS;

	/* return structure */
	return other;
}


/******************************************************************************
Specify that we want the "other" properties of an element to be tucked
away within the user's structure.  The user needn't be concerned for how
these properties are stored.

Entry:
elem_name - name of element that we want to store other_props in
offset    - offset to where other_props will be stored inside user's structure

Exit:
returns pointer to structure containing description of other_props
******************************************************************************/

PLY::PlyOtherProp* PLY::get_other_properties(
	const char *elem_name,
	int offset
	)
{
	/* find information about the element */
	PlyElement *elem = find_element(elem_name);
	if (elem == NULL) {
		DEBUG("warning: get_other_properties: Can't find element '%s'", elem_name);
		return (NULL);
	}

	PlyOtherProp *other = get_other_properties(elem, offset);
	return (other);
}




/*************************/
/*  Other Element Stuff  */
/*************************/





/******************************************************************************
Grab all the data for the current element that a user does not want to
explicitly read in.  Stores this in the PLY object's data structure.

Entry:

Exit:
returns pointer to ALL the "other" element data for this PLY file
******************************************************************************/

PLY::PlyOtherElems* PLY::get_other_element()
{
	PlyElement *elem = this->which_elem;

	/* create room for the new "other" element, initializing the */
	/* other data structure if necessary */
	OtherElem other;

	/* count of element instances in file */
	other.elem_count = elem->num;

	/* save name of element */
	other.elem_name = elem->name;

	/* create a list to hold all the current elements */
	other.other_data = new OtherData*[other.elem_count];

	/* set up for getting elements */
	other.other_props = get_other_properties(elem->name.c_str(), offsetof(OtherData,other_props));

	/* grab all these elements */
	for (int i = 0; i < other.elem_count; ++i) {
		/* grab and element from the file */
		other.other_data[i] = new OtherData;
		get_element((uint8_t*)other.other_data[i]);
	}

	/* return pointer to the other elements data */
	if (other_elems == NULL)
		other_elems = new PlyOtherElems;
	other_elems->other_list.push_back(other);
	return (other_elems);
}


/******************************************************************************
Write out the "other" elements specified for this PLY file.

Entry:
******************************************************************************/

void PLY::put_other_elements()
{
	/* make sure we have other elements to write */
	if (this->other_elems == NULL)
		return;

	/* write out the data for each "other" element */
	for (size_t i = 0; i < this->other_elems->other_list.size(); ++i) {
		OtherElem *other = &(this->other_elems->other_list[i]);
		put_element_setup(other->elem_name.c_str());

		/* write out each instance of the current element */
		for (int j = 0; j < other->elem_count; ++j)
			put_element(other->other_data[j]);
	}
}



/*******************/
/*  Miscellaneous  */
/*******************/


/******************************************************************************
Flush a PLY file.
******************************************************************************/

void PLY::flush()
{
	if (ostream != NULL) {
		if (mfp != NULL) {
			ostream->write(mfp->getBuffer(), mfp->getSize());
			mfp->setSize(0);
		}
		ostream->flush();
	}
}


/******************************************************************************
Use old PLY type names during writing for backward compatibility.
******************************************************************************/

void PLY::set_legacy_type_names()
{
	write_type_names = old_type_names;
}


/******************************************************************************
Get version number and file type of a PlyFile.

Entry:

Exit:
version - version of the file
file_type - PLY_ASCII, PLY_BINARY_BE, or PLY_BINARY_LE
******************************************************************************/

void PLY::get_info(float *version, int *file_type)
{
	*version = this->version;
	*file_type = this->file_type;
}


/******************************************************************************
Find an element from the element list of a given PLY object.

Entry:
element - name of element we're looking for

Exit:
returns the element, or NULL if not found
******************************************************************************/

PLY::PlyElement* PLY::find_element(const char *element) const
{
	for (size_t i=0; i<elems.size(); ++i)
		if (equal_strings(element, elems[i]->name.c_str()))
			return elems[i];
	return NULL;
}


/******************************************************************************
Find a property in the list of properties of a given element.

Entry:
elem      - pointer to element in which we want to find the property
prop_name - name of property to find

Exit:
returns the index to position in list
******************************************************************************/

int PLY::find_property(PlyElement *elem, const char *prop_name) const
{
	for (size_t i=0; i<elem->props.size(); ++i)
		if (equal_strings(prop_name, elem->props[i]->name.c_str()))
			return (int)i;
	return -1;
}


/******************************************************************************
Read an element from an ascii file.

Entry:
elem_ptr - pointer to element
******************************************************************************/

void PLY::ascii_get_element(uint8_t* elem_ptr)
{
	char *elem_data, *item;
	char *item_ptr;
	ValueType val;
	char *orig_line;
	char *other_data(NULL);
	int other_flag(0);

	/* the kind of element we're reading currently */
	PlyElement *elem = this->which_elem;

	/* do we need to setup for other_props? */
	if (elem->other_offset != NO_OTHER_PROPS) {
		other_flag = 1;
		/* make room for other_props */
		other_data = new char[elem->other_size];
		/* store pointer in user's structure to the other_props */
		*((char**)(elem_ptr + elem->other_offset)) = other_data;
	}

	/* read in the element */
	int nwords;
	char **words;
	{
	STRISTREAM sfp(istream);
	words = get_words(sfp, &nwords, &orig_line);
	if (words == NULL)
		abort_ply("error: get_element: unexpected end of file");
	sfp.emptyBuffer();
	}
	int which_word = 0;

	for (size_t j = 0; j < elem->props.size(); ++j) {
		PlyProperty *prop = elem->props[j];
		const int store_it(elem->store_prop[j] | other_flag);

		/* store either in the user's structure or in other_props */
		if (elem->store_prop[j])
			elem_data = (char*)elem_ptr;
		else
			elem_data = other_data;

		if (prop->is_list == LIST) {       /* a list */
			/* get and store the number of items in the list */
			get_ascii_item(words[which_word++], prop->count_external, val);
			if (store_it) {
				item = elem_data + prop->count_offset;
				store_item(item, prop->count_internal, val, prop->count_external);
			}

			/* allocate space for an array of items and store a ptr to the array */
			const int list_count(ValueType2Type<int>(val, prop->count_external));
			char **store_array = (char**)(elem_data + prop->offset);
			if (list_count == 0) {
				if (store_it)
					*store_array = NULL;
			}
			else {
				const int item_size(ply_type_size[prop->internal_type]);

				if (store_it) {
					item_ptr = new char[item_size * list_count];
					item = item_ptr;
					*store_array = item_ptr;
				}

				/* read items and store them into the array */
				for (int k = 0; k < list_count; k++) {
					get_ascii_item(words[which_word++], prop->external_type, val);
					if (store_it) {
						store_item(item, prop->internal_type, val, prop->external_type);
						item += item_size;
					}
				}
			}

		}
		else if (prop->is_list == STRING) {   /* a string */
			if (store_it) {
				item = elem_data + prop->offset;
				*((char **)item) = strdup(words[which_word++]);
			}
			else {
				which_word++;
			}
		}
		else {                               /* a scalar */
			get_ascii_item(words[which_word++], prop->external_type, val);
			if (store_it) {
				item = elem_data + prop->offset;
				store_item(item, prop->internal_type, val, prop->external_type);
			}
		}
	}

	free(words);
}


/******************************************************************************
Read an element from a binary file.

Entry:
elem_ptr - pointer to an element
******************************************************************************/

void PLY::binary_get_element(uint8_t* elem_ptr)
{
	char *elem_data;
	char *item;
	char *item_ptr;
	ValueType val;
	char *other_data(NULL);
	int other_flag(0);

	/* the kind of element we're reading currently */
	PlyElement *elem = this->which_elem;

	/* do we need to setup for other_props? */
	if (elem->other_offset != NO_OTHER_PROPS) {
		other_flag = 1;
		/* make room for other_props */
		other_data = new char[elem->other_size];
		/* store pointer in user's structure to the other_props */
		*((char **)(elem_ptr + elem->other_offset)) = other_data;
	}

	/* read in a number of elements */
	for (size_t j = 0; j < elem->props.size(); ++j) {
		PlyProperty *prop = elem->props[j];
		const int store_it(elem->store_prop[j] | other_flag);

		/* store either in the user's structure or in other_props */
		if (elem->store_prop[j])
			elem_data = (char*)elem_ptr;
		else
			elem_data = other_data;

		if (prop->is_list == LIST) {          /* list */

			/* get and store the number of items in the list */
			get_binary_item(istream, prop->count_external, val);
			if (store_it) {
				item = elem_data + prop->count_offset;
				store_item(item, prop->count_internal, val, prop->count_external);
			}

			/* allocate space for an array of items and store a ptr to the array */
			const int list_count(ValueType2Type<int>(val, prop->count_external));
			const int item_size(ply_type_size[prop->internal_type]);
			char **store_array = (char**)(elem_data + prop->offset);
			if (list_count == 0) {
				if (store_it)
					*store_array = NULL;
			}
			else {
				if (store_it) {
					item_ptr = new char[item_size * list_count];
					item = item_ptr;
					*store_array = item_ptr;
				}

				/* read items and store them into the array */
				for (int k = 0; k < list_count; k++) {
					get_binary_item(istream, prop->external_type, val);
					if (store_it) {
						store_item(item, prop->internal_type, val, prop->external_type);
						item += item_size;
					}
				}
			}

		}
		else if (prop->is_list == STRING) {     /* string */
			int len;
			istream->read(&len, sizeof(int));
			char *str = new char[len];
			istream->read(str, len);
			if (store_it) {
				item = elem_data + prop->offset;
				*((char **)item) = str;
			}
		}
		else {                                   /* scalar */
			get_binary_item(istream, prop->external_type, val);
			if (store_it) {
				item = elem_data + prop->offset;
				store_item(item, prop->internal_type, val, prop->external_type);
			}
		}
	}
}


/******************************************************************************
Write to a file the word that represents a PLY data type.

Entry:
fp   - file pointer
code - code for type
******************************************************************************/

void PLY::write_scalar_type(OSTREAM* fp, int code)
{
	/* make sure this is a valid code */
	if (code <= StartType || code >= EndType)
		abort_ply("error: write_scalar_type: bad data code = %d", code);

	/* write the code to a file */
	fp->print("%s", write_type_names[code]);
}


/******************************************************************************
Get a text line from a file and break it up into words.

IMPORTANT: The calling routine should call "free" on the returned pointer once
finished with it.

Entry:
sfp       - file to read from

Exit:
nwords    - number of words returned
orig_line - the original line of characters
returns a list of words from the line, or NULL if end-of-file
******************************************************************************/

char** PLY::get_words(STRISTREAM& sfp, int *nwords, char **orig_line)
{
	const int BIG_STRING = 4096;
	char str[BIG_STRING];
	char str_copy[BIG_STRING];

	int max_words = 10;
	int num_words = 0;
	char *ptr, *ptr2;

	char **words = (char **)malloc(sizeof (char *) * max_words);

	/* read in a line */
	size_t len(sfp.readLine(str, BIG_STRING-2));
	if (len == 0 || len == STREAM_ERROR) {
		*nwords = 0;
		*orig_line = NULL;
		free(words);
		return (NULL);
	}

	/* convert line-feed and tabs into spaces */
	/* (this guarantees that there will be a space before the */
	/*  null character at the end of the string) */
	if (str[len-1] == '\r')
		--len;
	str[len] = '\n';

	for (ptr = str, ptr2 = str_copy; ; ptr++, ptr2++) {
		switch (*ptr) {
		case '\t':
			*ptr = ' ';
			*ptr2 = ' ';
			break;
		case '\n':
			*ptr = ' ';
			*(ptr+1) = *ptr2 = '\0';
			goto EXIT_LOOP;
		default:
			*ptr2 = *ptr;
		}
	}
	EXIT_LOOP:

	/* find the words in the line */
	ptr = str;
	while (*ptr != '\0') {

		/* jump over leading spaces */
		while (*ptr == ' ')
			ptr++;

		/* break if we reach the end */
		if (*ptr == '\0')
			break;

		/* allocate more room for words if necessary */
		if (num_words >= max_words) {
			max_words += 10;
			words = (char **) realloc (words, sizeof (char *) * max_words);
		}

		if (*ptr == '\"') {  /* a quote indicates that we have a string */

			/* skip over leading quote */
			ptr++;

			/* save pointer to beginning of word */
			words[num_words++] = ptr;

			/* find trailing quote or end of line */
			while (*ptr != '\"' && *ptr != '\0')
				ptr++;

			/* replace quote with a null character to mark the end of the word */
			/* if we are not already at the end of the line */
			if (*ptr != '\0')
				*ptr++ = '\0';
		}
		else {               /* non-string */

			/* save pointer to beginning of word */
			words[num_words++] = ptr;

			/* jump over non-spaces */
			while (*ptr != ' ')
				ptr++;

			/* place a null character here to mark the end of the word */
			*ptr++ = '\0';
		}
	}

	/* return the list of words */
	*nwords = num_words;
	*orig_line = str_copy;
	return (words);
}


/******************************************************************************
Write out an item to a file as raw binary bytes.

Entry:
val        - item value to be written
double_val - value type
type       - data type to write out
******************************************************************************/

void PLY::write_binary_item(
	const ValueType& val,
	int from_type,
	int to_type
	)
{
	switch (to_type) {
	case Int8: {
		const int8_t v(ValueType2Type<int8_t>(val, from_type));
		f->write(&v, 1);
		break; }
	case Int16: {
		const int16_t v(ValueType2Type<int16_t>(val, from_type));
		f->write(&v, 2);
		break; }
	case Int32: {
		const int32_t v(ValueType2Type<int32_t>(val, from_type));
		f->write(&v, 4);
		break; }
	case Uint8: {
		const uint8_t v(ValueType2Type<uint8_t>(val, from_type));
		f->write(&v, 1);
		break; }
	case Uint16: {
		const uint16_t v(ValueType2Type<uint16_t>(val, from_type));
		f->write(&v, 2);
		break; }
	case Uint32: {
		const uint32_t v(ValueType2Type<uint32_t>(val, from_type));
		f->write(&v, 4);
		break; }
	case Float32: {
		const float v(ValueType2Type<float>(val, from_type));
		f->write(&v, 4);
		break; }
	case Float64: {
		const double v(ValueType2Type<double>(val, from_type));
		f->write(&v, 8);
		break; }
	default:
		abort_ply("error: write_binary_item: bad type = %d", to_type);
	}
}


/******************************************************************************
Write out an item to a file as ascii characters.

Entry:
val        - item value to be written
double_val - value type
type       - data type to write out
******************************************************************************/

void PLY::write_ascii_item(
	const ValueType& val,
	int from_type,
	int to_type
	)
{
	switch (to_type) {
	case Int8:
	case Int16:
	case Int32:
		f->print("%d ", ValueType2Type<int32_t>(val, from_type));
		break;
	case Uint8:
	case Uint16:
	case Uint32:
		f->print("%u ", ValueType2Type<uint32_t>(val, from_type));
		break;
	case Float32:
	case Float64:
		f->print("%g ", ValueType2Type<double>(val, from_type));
		break;
	default:
		abort_ply("error: write_ascii_item: bad type = %d", to_type);
	}
}


/******************************************************************************
Get the value of an item that is in memory, and place the result
into an integer, an unsigned integer and a double.

Entry:
ptr        - pointer to the item
type       - data type supposedly in the item

Exit:
val        - extracted value
******************************************************************************/

void PLY::get_stored_item(
	void *ptr,
	int type,
	ValueType& val
	)
{
	switch (type) {
	case Int8:
		val.i8 = *((int8_t*)ptr);
		break;
	case Uint8:
		val.u8 = *((uint8_t*)ptr);
		break;
	case Int16:
		val.i16 = *((int16_t*)ptr);
		break;
	case Uint16:
		val.u16 = *((uint16_t*)ptr);
		break;
	case Int32:
		val.i32 = *((int32_t*)ptr);
		break;
	case Uint32:
		val.u32 = *((uint32_t*)ptr);
		break;
	case Float32:
		val.f = *((float*)ptr);
		break;
	case Float64:
		val.d = *((double*)ptr);
		break;
	default:
		abort_ply("error: get_stored_item: bad type = %d", type);
	}
}


/******************************************************************************
Get the value of an item from a binary file, and place the result
into an integer, an unsigned integer and a double.

Entry:
fp         - file to get item from
type       - data type supposedly in the word

Exit:
val        - store value
******************************************************************************/

void PLY::get_binary_item(
	ISTREAM* fp,
	int type,
	ValueType& val
	)
{
	switch (type) {
	case Int8:
		fp->read(&val.i8, 1);
		break;
	case Uint8:
		fp->read(&val.u8, 1);
		break;
	case Int16:
		fp->read(&val.i16, 2);
		break;
	case Uint16:
		fp->read(&val.u16, 2);
		break;
	case Int32:
		fp->read(&val.i32, 4);
		break;
	case Uint32:
		fp->read(&val.u32, 4);
		break;
	case Float32:
		fp->read(&val.f, 4);
		break;
	case Float64:
		fp->read(&val.d, 8);
		break;
	default:
		abort_ply("error: get_binary_item: bad type = %d", type);
	}
}


/******************************************************************************
Extract the value of an item from an ascii word, and place the result
into an integer, an unsigned integer and a double.

Entry:
word       - word to extract value from
type       - data type supposedly in the word

Exit:
val        - store value
******************************************************************************/

void PLY::get_ascii_item(
	const char* word,
	int type,
	ValueType& val
	)
{
	switch (type) {
	case Int8:
		val.i8 = (int8_t)atoi(word);
		break;
	case Uint8:
		val.u8 = (uint8_t)atoi(word);
		break;
	case Int16:
		val.i16 = (int16_t)atoi(word);
		break;
	case Uint16:
		val.u16 = (uint16_t)atoi(word);
		break;
	case Int32:
		val.i32 = atoi(word);
		break;
	case Uint32:
		val.u32 = strtoul(word, (char **)NULL, 10);
		break;
	case Float32:
		val.f = (float)atof(word);
		break;
	case Float64:
		val.d = atof(word);
		break;
	default:
		abort_ply("error: get_ascii_item: bad type = %d", type);
	}
}


/******************************************************************************
Store a value into a place being pointed to, guided by a data type.

Entry:
to_type    - data type
val        - value to be stored
from_type  - value type

Exit:
ptr        - data pointer to stored value
******************************************************************************/

void PLY::store_item(
	void* ptr,
	int to_type,
	const ValueType& val,
	int from_type
	)
{
	switch (to_type) {
	case Int8:
		*((int8_t*)ptr) = ValueType2Type<int8_t>(val, from_type);
		break;
	case Uint8:
		*((uint8_t*)ptr) = ValueType2Type<uint8_t>(val, from_type);
		break;
	case Int16:
		*((int16_t*)ptr) = ValueType2Type<int16_t>(val, from_type);
		break;
	case Uint16:
		*((uint16_t*)ptr) = ValueType2Type<uint16_t>(val, from_type);
		break;
	case Int32:
		*((int32_t*)ptr) = ValueType2Type<int32_t>(val, from_type);
		break;
	case Uint32:
		*((uint32_t*)ptr) = ValueType2Type<uint32_t>(val, from_type);
		break;
	case Float32:
		*((float*)ptr) = ValueType2Type<float>(val, from_type);
		break;
	case Float64:
		*((double*)ptr) = ValueType2Type<double>(val, from_type);
		break;
	default:
		abort_ply("error: store_item: bad type = %d", to_type);
	}
}


/******************************************************************************
Add an element to a PLY file descriptor.

Entry:
words   - list of words describing the element
nwords  - number of words in the list
******************************************************************************/

void PLY::add_element(const char **words, int /*nwords*/)
{
	/* create the new element */
	PlyElement *elem = new PlyElement;
	elem->name = words[1];
	elem->num = atoi(words[2]);

	/* add the new element to the object's list */
	this->elems.push_back(elem);
}


/******************************************************************************
Return the type of a property, given the name of the property.

Entry:
name - name of property type

Exit:
returns integer code for property, or 0 if not found
******************************************************************************/

int PLY::get_prop_type(const char *type_name)
{
	/* try to match the type name */
	for (int i = StartType + 1; i < EndType; ++i)
		if (equal_strings (type_name, type_names[i]))
			return (i);

	/* see if we can match an old type name */
	for (int i = StartType + 1; i < EndType; ++i)
		if (equal_strings (type_name, old_type_names[i]))
			return (i);

	/* if we get here, we didn't find the type */
	return (0);
}


/******************************************************************************
Add a property to a PLY file descriptor.

Entry:
words   - list of words describing the property
nwords  - number of words in the list
******************************************************************************/

void PLY::add_property(const char **words, int /*nwords*/)
{
	/* create the new property */
	PlyProperty *prop = new PlyProperty;

	if (equal_strings(words[1], "list")) {          /* list */
		prop->count_external = get_prop_type (words[2]);
		prop->external_type = get_prop_type (words[3]);
		prop->name = words[4];
		prop->is_list = LIST;
	}
	else if (equal_strings(words[1], "string")) {   /* string */
		prop->count_external = Int8;
		prop->external_type = Int8;
		prop->name = words[2];
		prop->is_list = STRING;
	}
	else {                                           /* scalar */
		prop->external_type = get_prop_type (words[1]);
		prop->name = words[2];
		prop->is_list = SCALAR;
	}

	/* internal types are the same as external by default */
	prop->internal_type = prop->external_type;
	prop->count_internal = prop->count_external;

	/* add this property to the list of properties of the current element */
	PlyElement *elem = this->elems.back();
	elem->props.push_back(prop);
}


/******************************************************************************
Add a comment to a PLY file descriptor.

Entry:
line    - line containing comment
******************************************************************************/

void PLY::add_comment(const char *line)
{
	/* skip over "comment" and leading spaces and tabs */
	int i = 7;
	while (line[i] == ' ' || line[i] == '\t')
		i++;
	append_comment(&line[i]);
}


/******************************************************************************
Add a some object information to a PLY file descriptor.

Entry:
line    - line containing text info
******************************************************************************/

void PLY::add_obj_info(const char *line)
{
	/* skip over "obj_info" and leading spaces and tabs */
	int i = 8;
	while (line[i] == ' ' || line[i] == '\t')
		i++;
	append_obj_info(&line[i]);
}


/******************************************************************************
Copy a property.
******************************************************************************/

void PLY::copy_property(PlyProperty& dest, const PlyProperty& src)
{
	dest.name = src.name;
	dest.external_type = src.external_type;
	dest.internal_type = src.internal_type;
	dest.offset = src.offset;

	dest.is_list = src.is_list;
	dest.count_external = src.count_external;
	dest.count_internal = src.count_internal;
	dest.count_offset = src.count_offset;
}



/******************************************************************************
Return a list of the names of the elements in a particular PLY file.

Entry:

Exit:
elem_names - the list of element names
returns the number of elements
******************************************************************************/

int PLY::get_element_list(std::vector<std::string>& elem_names) const
{
	/* create the list of element names */
	elem_names.resize(elems.size());
	for (size_t i = 0; i < elems.size(); ++i)
		elem_names[i] = elems[i]->name;
	/* return the number of elements and the list of element names */
	return (int)elems.size();
}


/******************************************************************************
Append a comment to a PLY file.

Entry:
comment - the comment to append
******************************************************************************/

void PLY::append_comment(const char *comment)
{
	/* add comment to list */
	comments.push_back(comment);
}


/******************************************************************************
Copy the comments from one PLY file to another.

Entry:
out_ply - destination file to copy comments to
in_ply  - the source of the comments
******************************************************************************/

void PLY::copy_comments(const PLY& in_ply)
{
	for (size_t i = 0; i < in_ply.comments.size(); ++i)
		append_comment(in_ply.comments[i].c_str());
}


/******************************************************************************
Append object information (arbitrary text) to a PLY file.

Entry:
obj_info - the object info to append
******************************************************************************/

void PLY::append_obj_info(const char *obj_info)
{
	/* add info to list */
	this->obj_info.push_back(obj_info);
}


/******************************************************************************
Copy the object information from one PLY file to another.

Entry:
out_ply - destination file to copy object information to
in_ply  - the source of the object information
******************************************************************************/

void PLY::copy_obj_info(const PLY& in_ply)
{
	for (size_t i = 0; i < in_ply.obj_info.size(); ++i)
		append_obj_info(in_ply.obj_info[i].c_str());
}


/******************************************************************************
Specify the index of the next element to be read in from a PLY file.

Entry:
index - index of the element to be read

Exit:
elem_count - the number of elements in the file
returns pointer to the name of this next element
******************************************************************************/

LPCSTR PLY::setup_element_read(int index, int* elem_count)
{
	if ((size_t)index > elems.size()) {
		DEBUG("warning:  No element with index %d", index);
		return (0);
	}

	PlyElement* elem = elems[index];

	/* set this to be the current element */
	which_elem = elem;

	/* return the number of such elements in the file and the element's name */
	*elem_count = elem->num;
	return elem->name.c_str();
}


/******************************************************************************
Specify one of several properties of the current element that is to be
read from a file.  This should be called (usually multiple times) before a
call to the routine get_element().

Entry:
prop    - property to add to those that will be returned
******************************************************************************/

void PLY::setup_property(const PlyProperty& prop)
{
	PlyElement *elem = this->which_elem;

	/* deposit the property information into the element's description */
	int index = find_property(elem, prop.name.c_str());
	if (index == -1) {
		DEBUG("warning:  Can't find property '%s' in element '%s'", prop.name.c_str(), elem->name.c_str());
		return;
	}
	PlyProperty *prop_ptr = elem->props[index];
	prop_ptr->internal_type  = prop.internal_type;
	prop_ptr->offset         = prop.offset;
	prop_ptr->count_internal = prop.count_internal;
	prop_ptr->count_offset   = prop.count_offset;

	/* specify that the user wants this property */
	elem->store_prop[index] = STORE_PROP;
}


/******************************************************************************
Specify that we want the "other" properties of the current element to be tucked
away within the user's structure.

Entry:
offset  - offset to where other_props will be stored inside user's structure

Exit:
returns pointer to structure containing description of other_props
******************************************************************************/

PLY::PlyOtherProp* PLY::get_other_properties(int offset)
{
	return get_other_properties(this->which_elem, offset);
}


/******************************************************************************
Describe which element is to be written next and state how many of them will
be written.

Entry:
elem_name - name of element that information is being described
nelems    - number of elements of this type to be written
******************************************************************************/

void PLY::describe_element(
	char *elem_name,
	int nelems
	)
{
	/* look for appropriate element */
	PlyElement *elem = find_element(elem_name);
	if (elem == NULL)
		abort_ply("error: describe_element: can't find element '%s'",elem_name);

	elem->num = nelems;

	/* now this element is the current element */
	this->which_elem = elem;
}


/******************************************************************************
Describe a property of an element.

Entry:
prop      - the new property
******************************************************************************/

void PLY::describe_property(const PlyProperty& prop)
{
	PlyElement *elem = this->which_elem;

	/* copy the new property */
	PlyProperty *elem_prop = new PlyProperty;
	copy_property(*elem_prop, prop);
	elem->props.push_back(elem_prop);
	elem->store_prop.push_back(NAMED_PROP);
}


/******************************************************************************
Describe what the "other" properties are that are to be stored, and where
they are in an element.
******************************************************************************/

void PLY::describe_other_properties(
	PlyOtherProp *other,
	int offset
	)
{
	/* look for appropriate element */
	PlyElement *elem = find_element(other->name.c_str());
	if (elem == NULL) {
		DEBUG("warning: describe_other_properties: can't find element '%s'", other->name.c_str());
		return;
	}

	/* copy the other properties */
	for (size_t i = 0; i < other->props.size(); ++i) {
		PlyProperty *prop = new PlyProperty;
		copy_property(*prop, *other->props[i]);
		elem->props.push_back(prop);
		elem->store_prop.push_back(OTHER_PROP);
	}

	/* save other info about other properties */
	elem->other_size = other->size;
	elem->other_offset = offset;
}


/******************************************************************************
Pass along a pointer to "other" elements that we want to save in a given
PLY file.  These other elements were presumably read from another PLY file.

Entry:
other_elems - info about other elements that we want to store
******************************************************************************/

void PLY::describe_other_elements(PlyOtherElems *other_elems)
{
	/* ignore this call if there is no other element */
	if (other_elems == NULL)
		return;

	/* save pointer to this information */
	this->other_elems = other_elems;

	/* describe the other properties of this element */
	for (size_t i = 0; i < other_elems->other_list.size(); ++i) {
		OtherElem *other = &(other_elems->other_list[i]);
		element_count(other->elem_name.c_str(), other->elem_count);
		describe_other_properties(other->other_props, offsetof(OtherData,other_props));
	}
}



/******************************************************************************
Initialize the property propagation rules for an element.  Default is to
use averaging (AVERAGE_RULE) for creating all new properties.

Entry:
elem_name - name of the element that we're making the rules for

Exit:
returns pointer to the default rules
******************************************************************************/

PLY::PlyPropRules* PLY::init_rule(const char *elem_name)
{
	PlyElement *elem = find_element(elem_name);
	if (elem == NULL)
		abort_ply("error: init_rule: Can't find element '%s'", elem_name);

	PlyPropRules *rules = new PlyPropRules;
	rules->elem = elem;
	rules->max_props = 0;
	rules->rule_list = NULL;

	/* see if there are other rules we should use */
	if (elem->props.empty())
		return (rules);

	/* default is to use averaging rule */
	rules->rule_list = new int[elem->props.size()];
	for (size_t i = 0; i < elem->props.size(); ++i)
		rules->rule_list[i] = AVERAGE_RULE;

	/* try to match the element, property and rule name */
	for (PlyRuleList *list = this->rule_list; list != NULL; list = list->next) {

		if (!equal_strings(list->element, elem->name.c_str()))
			continue;

		int found_prop = 0;
		for (size_t i = 0; i < elem->props.size(); ++i)
			if (equal_strings(list->property, elem->props[i]->name.c_str())) {

				found_prop = 1;

				/* look for matching rule name */
				for (int j = 0; rule_name_list[j].code != -1; ++j)
					if (equal_strings(list->name, rule_name_list[j].name.c_str())) {
						rules->rule_list[i] = rule_name_list[j].code;
						break;
					}
			}

		if (!found_prop) {
			DEBUG("warning: Can't find property '%s' for rule '%s'", list->property, list->name);
			continue;
		}
	}

	return (rules);
}


/******************************************************************************
Modify a property propagation rule.

Entry:
rules - rules for the element
prop_name - name of the property whose rule we're modifying
rule_type - type of rule (MAXIMUM_RULE, MINIMUM_RULE, MAJORITY_RULE, etc.)
******************************************************************************/

void PLY::modify_rule(PlyPropRules *rules, const char *prop_name, int rule_type)
{
	PlyElement *elem = rules->elem;

	/* find the property and modify its rule type */
	for (size_t i = 0; i < elem->props.size(); ++i)
		if (equal_strings(elem->props[i]->name.c_str(), prop_name)) {
			rules->rule_list[i] = rule_type;
			return;
		}

		/* we didn't find the property if we get here */
		abort_ply("error: modify_rule: Can't find property '%s'", prop_name);
}


/******************************************************************************
Begin to create a set of properties from a set of propagation rules.

Entry:
rules - rules for the element
******************************************************************************/

void PLY::start_props(PlyPropRules *rules)
{
	/* save pointer to the rules in the PLY object */
	this->current_rules = rules;
}


/******************************************************************************
Remember a set of properties and their weights for creating a new set of
properties.

Entry:
weight      - weights for this set of properties
other_props - the properties to use
******************************************************************************/

void PLY::weight_props(float weight, void *other_props)
{
	PlyPropRules *rules = this->current_rules;

	/* allocate space for properties and weights, if necessary */
	if (rules->max_props == 0) {
		rules->max_props = 6;
	}
	if (rules->props.size() == rules->max_props) {
		rules->max_props *= 2;
	}
	rules->props.reserve(rules->max_props);
	rules->weights.reserve(rules->max_props);

	/* remember these new properties and their weights */
	rules->props.push_back(other_props);
	rules->weights.push_back(weight);
}


/******************************************************************************
Return a pointer to a new set of properties that have been created using
a specified set of property combination rules and a given collection of
"other" properties.

Exit:
returns a pointer to the new properties
******************************************************************************/

void* PLY::get_new_props()
{
	PlyPropRules *rules = this->current_rules;
	PlyElement *elem = rules->elem;
	PlyProperty *prop;
	int offset;
	int type;
	ValueType val;
	int random_pick;

	/* return NULL if we've got no "other" properties */
	if (elem->other_size == 0)
		return (NULL);

	/* create room for combined other properties */
	char *new_data = new char[elem->other_size];

	/* make sure there is enough room to store values we're to combine */
	vals.resize(rules->props.size());

	/* in case we need a random choice */
	random_pick = FLOOR2INT(SEACAVE::random() * rules->props.size());

	/* calculate the combination for each "other" property of the element */
	for (size_t i = 0; i < elem->props.size(); ++i) {

		/* don't bother with properties we've been asked to store explicitly */
		if (elem->store_prop[i])
			continue;

		prop = elem->props[i];
		offset = prop->offset;
		type = prop->external_type;

		/* collect together all the values we're to combine */
		for (size_t j = 0; j < rules->props.size(); ++j) {
			char* data = (char *)rules->props[j];
			void* ptr = (void *)(data + offset);
			get_stored_item((void*)ptr, type, val);
			vals[j] = ValueType2Type<double>(val, type);
		}

		/* calculate the combined value */
		switch (rules->rule_list[i]) {
		case AVERAGE_RULE: {
			double sum = 0;
			double weight_sum = 0;
			for (size_t j = 0; j < rules->props.size(); ++j) {
				sum += vals[j] * rules->weights[j];
				weight_sum += rules->weights[j];
			}
			val.d = sum / weight_sum;
			break;
						   }
		case MINIMUM_RULE: {
			val.d = vals[0];
			for (size_t j = 1; j < rules->props.size(); ++j)
				if (val.d > vals[j])
					val.d = vals[j];
			break;
						   }
		case MAXIMUM_RULE: {
			val.d = vals[0];
			for (size_t j = 1; j < rules->props.size(); ++j)
				if (val.d < vals[j])
					val.d = vals[j];
			break;
						  }
		case RANDOM_RULE: {
			val.d = vals[random_pick];
			break;
						  }
		case SAME_RULE: {
			val.d = vals[0];
			for (size_t j = 1; j < rules->props.size(); ++j)
				if (val.d != vals[j])
					abort_ply("error: get_new_props: Error combining properties that should be the same");
			break;
						}
		default:
			abort_ply("error: get_new_props: Bad rule = %d", rules->rule_list[i]);
		}

		/* store the combined value */
		store_item((void*)(new_data + offset), type, val, Float64);
	}

	return ((void*)new_data);
}


/******************************************************************************
Set the list of user-specified property combination rules.
******************************************************************************/

void PLY::set_prop_rules(PlyRuleList *prop_rules)
{
	this->rule_list = prop_rules;
}


/******************************************************************************
Append a property rule to a growing list of user-specified rules.

Entry:
rule_list - current rule list
name      - name of property combination rule
property  - "element.property" says which property the rule affects

Exit:
returns pointer to the new rule list
******************************************************************************/

PLY::PlyRuleList* PLY::append_prop_rule(
	PlyRuleList *rule_list,
	const char *name,
	const char *property
	)
{
	char *str2;
	char *ptr;

	/* find . */
	char *str = strdup(property);
	for (ptr = str; *ptr != '\0' && *ptr != '.'; ptr++) ;

	/* split string at . */
	if (*ptr == '.') {
		*ptr = '\0';
		str2 = ptr + 1;
	}
	else {
		DEBUG("warning: Can't find property '%s' for rule '%s'", property, name);
		return (rule_list);
	}

	PlyRuleList *rule = new PlyRuleList;
	rule->name = name;
	rule->element = str;
	rule->property = str2;
	rule->next = NULL;

	/* either start rule list or append to it */
	if (rule_list == NULL)
		rule_list = rule;
	else {                      /* append new rule to current list */
		PlyRuleList *rule_ptr = rule_list;
		while (rule_ptr->next != NULL)
			rule_ptr = rule_ptr->next;
		rule_ptr->next = rule;
	}

	/* return pointer to list */
	return (rule_list);
}


/******************************************************************************
See if a name matches the name of any property combination rules.

Entry:
name - name of rule we're trying to match

Exit:
returns 1 if we find a match, 0 if not
******************************************************************************/

int PLY::matches_rule_name(const char *name)
{
	for (int i = 0; rule_name_list[i].code != -1; ++i)
		if (equal_strings(rule_name_list[i].name.c_str(), name))
			return 1;
	return 0;
}
