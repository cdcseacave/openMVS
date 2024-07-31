/*
* ListFIFO.h
*
* Copyright (c) 2014-2024 SEACAVE
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* Additional Terms:
*
*      You are required to preserve legal notices and author attributions in
*      that material or in the Appropriate Legal Notices displayed by works
*      containing it.
*/

#pragma once
#ifndef _MVS_LISTFIFO_H_
#define _MVS_LISTFIFO_H_


// I N C L U D E S /////////////////////////////////////////////////

#include <list>
#include <unordered_map>


// S T R U C T S ///////////////////////////////////////////////////

namespace SEACAVE {

// Tracks accesses of some hash-able type T and records the least recently accessed.
template<typename T>
class ListFIFO {
public:
	// add or move an key to the front
	void Put(const T& key) {
		const auto it = map.find(key);
		if (it != map.end()) {
			// if key exists, remove it from its current position
			order.erase(it->second);
		}
		// add the key to the front
		order.push_front(key);
		map[key] = order.begin();
	}

	// remove and return the least used key (from the back)
	T Pop() {
		ASSERT(!IsEmpty());
		const T leastUsed = order.back();
		order.pop_back();
		map.erase(leastUsed);
		return leastUsed;
	}

	// return the least used key (from the back)
	const T& Back() {
		ASSERT(!IsEmpty());
		return order.back();
	}

	// check if the list is empty
	bool IsEmpty() const {
		return order.empty();
	}

	// get the size of the list
	size_t Size() const {
		return order.size();
	}

	// return true if the key is in the list
	bool Contains(const T& key) const {
		return map.find(key) != map.end();
	}

	// return the keys currently in cache
	const std::list<T>& GetCachedValues() const {
		return order;
	}

	// print the current order of elements
	void PrintOrder() const {
		std::cout << "Current order: ";
		for (const auto& element : order) {
			std::cout << element << " ";
		}
		std::cout << std::endl;
	}

private:
	std::list<T> order;
	std::unordered_map<T, typename std::list<T>::iterator> map;
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif
