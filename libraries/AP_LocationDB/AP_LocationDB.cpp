#include "AP_LocationDB.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <stdio.h>

#ifndef AP_LOCATIONDB_CAPACITY_DEFAULT
    #define AP_LOCATIONDB_CAPACITY_DEFAULT          100
#endif

#define AP_LOCATIONDB_TIMEOUT_DEFAULT 10000
#define AP_LOCATIONDB_CLEANUP_TIME_DEFAULT 30000
#define AP_LOCATIONDB_FLAGS_DEFAULT 1 // avoid only

const AP_Param::GroupInfo AP_LocationDB::var_info[] = {
    // @Param: SIZE
    // @DisplayName: Location Database maximum number of items
    // @Description: LocationDB maximum number of points. Set to 0 to disable the LocationDB. Larger means more points but is more cpu intensive to process
    // @Range: 0 10000
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("CAPACITY", 1, AP_LocationDB, _database_capacity_param, AP_LOCATIONDB_CAPACITY_DEFAULT),

    // @Param: TIMEOUT
    // @DisplayName: Location Database item timeout
    // @Description: Time since last update after which the position of a location database item if invalidated
    // @Range: 0 30000
    // @Unit: ms
    // @User: Advanced
    AP_GROUPINFO("TIMEOUT", 2, AP_LocationDB, _item_timeout, AP_LOCATIONDB_TIMEOUT_DEFAULT),

    // @Param: DEF_FLAGS
    // @DisplayName: Default flags for new database items
    // @Description: Default flags for new database items
    // @Bitmask: 1: AVOID
    // @User: Advanced
    AP_GROUPINFO("DEF_FLAGS", 3, AP_LocationDB, _default_flags, AP_LOCATIONDB_FLAGS_DEFAULT),

    // @Param: INC_RADIUS
    // @DisplayName: Location database radius of inclusion
    // @Description: Maximum distance of a database item from the vehicle to be included in location database. Beyond this distance, the items are excluded.
    // @Range: 0 10000
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("INC_RADIUS", 4, AP_LocationDB, _inc_radius, 100),

    AP_GROUPEND
};

AP_LocationDB::AP_LocationDB() {
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_LocationDB must be singleton");
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);

}

void AP_LocationDB::init() {

#if APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    _capacity = AP_LOCATIONDB_CAPACITY_DEFAULT;
# else
    _capacity = _database_capacity_param;
#endif

    if (_capacity == 0) {
        return;
    }

    _items = new DBItem[_capacity];
    clear();

    if (!healthy()) {
#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
        gcs().send_text(MAV_SEVERITY_INFO, "LocationDB init failed . DB size: %u", _capacity);
#endif
        delete[] _items;
        return;
    }

#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    gcs().send_text(MAV_SEVERITY_INFO, "LocationDB init successful . DB size: %u", _capacity);
#endif
}

void AP_LocationDB::update() {
    if (!healthy()) {
        return;
    }

    // invalidate fields of timed out items
    for (int i=0; i<_size; i++) {
        DBItem item;
        if (!get_item_at_index(i, item)) {
            continue;
        }

        const uint32_t time_since_last_update = AP_HAL::millis() - item.get_timestamp_ms();
        if ((time_since_last_update > (uint32_t)_item_timeout.get()) && item._populated_fields != 0) {
            item._populated_fields = 0;  // invalidate position
            update_item_at_index(i, item);
            char src_info[26];
            get_source_info(src_info, 26, item.get_key());
            gcs().send_text(MAV_SEVERITY_CRITICAL, "LocationDB item lost (Key: %u, %s)", item.get_key(), src_info);
        }
    }
}

bool AP_LocationDB::add_item(DBItem item) {
    if (!healthy() || is_full() || !is_valid_key(item.get_key())) {
        return false;
    }

#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    Vector3f item_pos;
    Vector3f our_pos;
    if (item.get_pos_NEU(item_pos) && AP::ahrs().get_relative_position_NED_origin(our_pos)) {
        our_pos.z = -our_pos.z;
        item_pos = item_pos / 100; // cm to m
        if ((item_pos - our_pos).length_squared() > (_inc_radius.get() * _inc_radius.get())) {
            // item very far away from the vehicle
            return false; 
        }
    }
#endif

    {
        WITH_SEMAPHORE(db_sem);
        uint16_t item_idx;
        if (get_item_index(item.get_key(), item_idx)) {
            return update_item_at_index(item_idx, item);
        } else {
            _items[_size] = item;
            _size += 1;
        }
    }

#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    char src_info[26];
    get_source_info(src_info, 26, item.get_key());
    gcs().send_text(MAV_SEVERITY_INFO, "LocationDB item added (Key: %u, %s)", item.get_key(), src_info);
#endif

    return true;
}

bool AP_LocationDB::get_item_index(const uint32_t key, uint16_t &index) {
    if (!healthy() || !is_valid_key(key)) {
        return false;
    }

    for (int i=0; i<_size; i++) {
        if (_items[i].get_key() == key) {
            index = i;
            return true;
        }
    }

    return false;
}

bool AP_LocationDB::item_exists(uint32_t key) {
    uint16_t idx;

    // make sure we are able  to retrive the item and some fields are populated
    // fields are not populated for timed out items
    if (get_item_index(key, idx) && _items[idx]._populated_fields != 0) {
        return true;
    }

    return false;
}

bool AP_LocationDB::get_item(const uint32_t key, DBItem &ret) {
    if (!healthy() || !is_valid_key(key)) {
        return false;
    }

    {
        WITH_SEMAPHORE(db_sem);
        uint16_t item_idx;
        if (get_item_index(key, item_idx)) {
            ret = _items[item_idx];
            return true;
        }
    }

    return false;
}

bool AP_LocationDB::get_item_at_index(const uint16_t index, DBItem &ret) {
    if (!healthy() || index >= _size) {
        return false;
    }

    {
        WITH_SEMAPHORE(db_sem);
        ret = _items[index];
    }

    return true;
}

bool AP_LocationDB::remove_item(const uint32_t key) {
    if (!healthy() || !is_valid_key(key)) {
        return false;
    }

    {
        WITH_SEMAPHORE(db_sem);
        uint16_t item_idx;
        if (get_item_index(key, item_idx)) {
             _size -= 1;

            if (_size != 0) {
                _items[item_idx] = _items[_size];
            }

            return true;
        }
    }

    return false;
}

bool AP_LocationDB::update_item_at_index(const uint16_t index, DBItem &new_item) {
    if (!healthy() || !is_valid_key(new_item.get_key()) || index >= _size) {
        return false;
    }

#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    bool was_lost = false;
    {
        WITH_SEMAPHORE(db_sem);
        was_lost = (_items[index]._populated_fields == 0); // if no fields were populated, that means the items was lost
        _items[index] = new_item;
    }

    if (was_lost) {
        // the old item was lost, we found it again, tell the user
        char src_info[26];
        get_source_info(src_info, 26, new_item.get_key());
        gcs().send_text(MAV_SEVERITY_INFO, "LocationDB item found (Key: %u, %s)", new_item.get_key(), src_info);
    }
#else
    {
        WITH_SEMAPHORE(db_sem);
        _items[index] = new_item;
    }
#endif

    return true;
}

bool AP_LocationDB::update_item(const uint32_t key, DBItem &new_item) {
    if (!healthy() || !is_valid_key(key)) {
        return false;
    }

#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    Vector3f pos;
    if (new_item.get_pos_NEU(pos) && pos.length_squared() > (_inc_radius.get() * 100 * _inc_radius.get() * 100)) {
        // item very far away from the vehicle
        return false;
    }
#endif

    {
        WITH_SEMAPHORE(db_sem);
        uint16_t item_idx;
        if (get_item_index(key, item_idx)) {
            return update_item_at_index(item_idx, new_item);
        }
    }

    return false;
}

bool AP_LocationDB::is_valid_key(uint32_t key) {
    const KeyDomain key_domain = static_cast<KeyDomain>(key >> 24);
    switch (key_domain) {
        case KeyDomain::MAVLINK:
            return (key & 255U) != 255U;  // last byte should not be 255, this means the mavlink msgid is not supported
        case KeyDomain::ADSB:
            return true;
    }

    return false;
}

uint32_t AP_LocationDB::construct_key_mavlink(uint8_t sysid, uint8_t compid, uint8_t msgid) {
    return (uint8_t)KeyDomain::MAVLINK << 24 | sysid << 16 | compid << 8 | short_mav_msg_id(msgid);
}

uint32_t AP_LocationDB::construct_key_adsb(uint32_t icao) {
    return (uint8_t)KeyDomain::ADSB << 24 | (icao & ((1U << 24) - 1U));
}

uint8_t AP_LocationDB::short_mav_msg_id(uint32_t msgid) {
    switch(msgid) {
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        return 0U;
    case MAVLINK_MSG_ID_FOLLOW_TARGET:
        return 1U;
    };

    // return 255 message not listed
    // is_key_valid() returns false for keys with this id
    return 255U;
}

void AP_LocationDB::get_source_info(char* ret, int len, const uint32_t key) const {
    if (!is_valid_key(key)) {
        return;
    }

    uint8_t domain = key >> 24;
    switch (domain) {
    case uint8_t(KeyDomain::ADSB):
        snprintf(ret, len, "Src: ADSB, ICAO: %u", (key & ((1 << 24) - 1)));
        break;
    case uint8_t(KeyDomain::MAVLINK):
        snprintf(ret, len, "Src: MAV, SYSID: %u", uint8_t(key >> 16));
        break;
    default:
        snprintf(ret, len, "UNKNOWN");
    }
}

void AP_LocationDB::clear() {
    _size = 0;
}

AP_LocationDB::DBItem::DBItem(uint32_t key, uint32_t timestamp_ms, Vector3f pos, Vector3f vel, Vector3f acc, float heading, float radius, uint8_t populated_fields) {
    _key = key;
    _timestamp_ms = timestamp_ms;
    _pos = pos;
    _vel = vel;
    _acc = acc;
    _heading = heading;
    _radius = radius;
    _populated_fields = populated_fields;

#if APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    _flags = AP_LOCATIONDB_FLAGS_DEFAULT; // avoid only
# else
    _flags = uint8_t(AP::locationdb()->_default_flags.get());
#endif
}

void AP_LocationDB::DBItem::init(uint32_t key, uint32_t timestamp_ms, Vector3f pos, Vector3f vel, Vector3f acc, float heading, float radius, uint8_t populated_fields) {
    _key = key;
    _timestamp_ms = timestamp_ms;
    _pos = pos;
    _vel = vel;
    _acc = acc;
    _heading = heading;
    _radius = radius;
    _populated_fields = populated_fields;

#if APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    _flags = AP_LOCATIONDB_FLAGS_DEFAULT; // avoid only
# else
    _flags = uint8_t(AP::locationdb()->_default_flags.get());
#endif
}


bool AP_LocationDB::DBItem::get_pos_NEU(Vector3f &ret) const {
    if (!is_field_populated(DataField::POS)) {
        return false;
    }

    ret = _pos;
    return true;
}

bool AP_LocationDB::DBItem::get_vel_NEU(Vector3f &ret) const {
    if (!is_field_populated(DataField::VEL)) {
        return false;
    }

    ret = _vel;
    return true;
}

bool AP_LocationDB::DBItem::get_acc_NEU(Vector3f &ret) const {
    if (!is_field_populated(DataField::ACC)) {
        return false;
    }

    ret = _acc;
    return true;
}

bool AP_LocationDB::DBItem::get_heading(float &ret) const {
    if (!is_field_populated(DataField::HEADING)) {
        return false;
    }

    ret = _heading;
    return true;
}

bool AP_LocationDB::DBItem::get_radius(float &ret) const {
    if (!is_field_populated(DataField::RADIUS)) {
        return false;
    }

    ret = _radius;
    return true;
}

bool AP_LocationDB::DBItem::is_flag_set(Flag flag) const {
    return (_flags & (uint8_t)flag) != 0;
}

void AP_LocationDB::DBItem::set_flag(Flag flag) {
    _flags |= (uint8_t)flag;
}

void AP_LocationDB::DBItem::unset_flag(Flag flag) {
    _flags &= ~((uint8_t)flag);
}

void AP_LocationDB::DBItem::copy_flags(uint8_t flags) {
    _flags = flags;
}

uint8_t AP_LocationDB::DBItem::get_flags() {
    return _flags;
}

// singleton instance
AP_LocationDB *AP_LocationDB::_singleton;

namespace AP {
    AP_LocationDB *locationdb() {
        return AP_LocationDB::get_singleton();
    }
}
