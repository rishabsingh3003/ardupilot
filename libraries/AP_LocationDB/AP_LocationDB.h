#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#define AP_LOCATIONDB_ITEM_SIZE 56

class AP_LocationDB {
public:

    AP_LocationDB();

    CLASS_NO_COPY(AP_LocationDB);

    static AP_LocationDB *get_singleton() {
        return _singleton;
    }

    void init();
    void update();

    enum class KeyDomain : uint8_t {
        // 0 should not be used
        MAVLINK = 1U,
        ADSB = 2U,
    };

    class DBItem {
    public:
        friend class AP_Filesystem_LocationDB;
        friend class AP_LocationDB;

        DBItem() {};
        DBItem(uint32_t key, uint32_t timestamp_ms, Vector3f pos, Vector3f vel, Vector3f acc, float heading, float radius, uint8_t populated_fields);

        void init(uint32_t key, uint32_t timestamp_ms, Vector3f pos, Vector3f vel, Vector3f acc, float heading, float radius, uint8_t populated_fields);

        uint32_t get_timestamp_ms() { return _timestamp_ms; }
        uint32_t get_key() { return _key; }
        bool get_pos_NEU(Vector3f &ret) const;
        bool get_vel_NEU(Vector3f &ret) const;
        bool get_acc_NEU(Vector3f &ret) const;
        bool get_heading(float &heading) const;
        bool get_radius(float &ret) const;

        enum class DataField : uint8_t {
            POS = (1U << 0),
            VEL = (1U << 1),
            ACC = (1U << 2),
            HEADING = (1U << 3),
            RADIUS = (1U << 4),
        };

        enum class Flag : uint8_t {
            AVOID = (1U << 0),
        };

        bool is_flag_set(Flag flag) const;
        void set_flag(Flag flag);
        void unset_flag(Flag flag);
        void copy_flags(uint8_t flags);
        uint8_t get_flags();

    private:
        uint32_t _key;
        uint32_t _timestamp_ms;
        Vector3f _pos;              // position of item in NEU
        Vector3f _vel;
        Vector3f _acc;
        float _heading;
        float _radius;
        uint8_t _flags;

        uint8_t _populated_fields = 0;

        bool is_field_populated(DataField option) const {
            return (_populated_fields & (uint8_t)option) != 0;
        }
    };

    bool add_item(DBItem item);
    bool get_item(const uint32_t key, DBItem &ret); // return first item with given key
    bool get_item_at_index(const uint16_t index, DBItem &ret);
    bool update_item(const uint32_t key, DBItem &new_item);
    bool remove_item(const uint32_t key);
    bool healthy() const { return _items != nullptr; }
    bool is_full() const { return _capacity == _size; };
    void clear();
    uint16_t size() const { return _size; }
    uint16_t capacity() const { return _capacity; }
    bool item_exists(uint32_t key); 
    
    static bool is_valid_key(uint32_t key);
    static uint32_t construct_key_mavlink(uint8_t sysid, uint8_t compid, uint8_t msgid);
    static uint32_t construct_key_adsb(uint32_t icao);
    static uint8_t short_mav_msg_id(uint32_t msgid);

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

    // parameters
    AP_Int16        _database_capacity_param;                   // db capacity
    AP_Int8         _default_flags;                             // default flags on new database item
    AP_Int16        _item_timeout;                              // timeout for db item
    AP_Int16        _inc_radius;                                // inclusion radius

private:
    assert_storage_size<DBItem, AP_LOCATIONDB_ITEM_SIZE> _assert_storage_size_DBItem UNUSED_PRIVATE_MEMBER;

    uint16_t _capacity = 0;
    uint16_t _size = 0;
    DBItem* _items = nullptr;
    HAL_Semaphore db_sem;
    bool get_item_index(const uint32_t key, uint16_t &index);
    bool update_item_at_index(const uint16_t index, DBItem &item);
    void get_source_info(char* ret, int len, const uint32_t key) const;

    static AP_LocationDB *_singleton;
};

namespace AP {
    AP_LocationDB *locationdb();
}
