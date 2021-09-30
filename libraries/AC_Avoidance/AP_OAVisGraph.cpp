/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_OAVisGraph.h"
#include <AP_Math/AP_Math.h>

// constructor initialises expanding array to use 20 elements per chunk
AP_OAVisGraph::AP_OAVisGraph() :
    _items(20)
{
}

// add item to visiblity graph, returns true on success, false if graph is full
bool AP_OAVisGraph::add_item(const OAItemID &id1, const OAItemID &id2, float distance_cm)
{
    // no more than 65k items
    if (_num_items == UINT16_MAX) {
        return false;
    }

    // ensure there is space in the array
    if (!_items.expand_to_hold(_num_items+1)) {
        return false;
    }

    // add item
    _items[_num_items] = {id1, id2, distance_cm};
    _num_items++;
    return true;
}


// add item to visiblity graph, returns true on success, false if graph is full
bool AP_OAVisGraph::add_item_to_buffer(uint16_t index, float distance_cm)
{
    // no more than 65k items
    if (_num_items_backup_buffer == UINT16_MAX) {
        return false;
    }

    // ensure there is space in the array
    if (!_items_backup_buffer.expand_to_hold(_num_items_backup_buffer+1)) {
        return false;
    }

    // add item
    _items_backup_buffer[_num_items_backup_buffer] = {index, distance_cm};
    _num_items_backup_buffer++;
    return true;
}

bool AP_OAVisGraph::empty_buffer()
{
    for (uint16_t i=0; i<_num_items_backup_buffer; i++) {
        _items[_items_backup_buffer[i].index].distance_cm = _items_backup_buffer[i].distance_cm;
    }
    _num_items_backup_buffer = 0;
}


bool AP_OAVisGraph::remove_item(const uint16_t index)
{
    if (index > _num_items) {
        // how is this possible
        return false;
    }
    _items[index].distance_cm = FLT_MAX;
}