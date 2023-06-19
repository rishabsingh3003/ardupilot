#include <AP_gtest.h>

/*
  tests for AP_LocationDB
 */

#include <AP_LocationDB/AP_LocationDB.h>
#include <GCS_MAVLink/GCS.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX

float rand_float(void) {
    return ((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6;
}

uint32_t create_random_key(AP_LocationDB::KeyDomain domain) {
    switch (domain) {
    case AP_LocationDB::KeyDomain::MAVLINK:{
        uint8_t sysid = ((unsigned)random() % 255U);
        uint8_t compid = ((unsigned)random() % 255U);
        uint8_t msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
        return AP_LocationDB::construct_key_mavlink(sysid, compid, msgid);
        break;
    }
    case AP_LocationDB::KeyDomain::ADSB:
        uint32_t icao = ((unsigned)random() % ((1U << 24) - 1U));
        return AP_LocationDB::construct_key_adsb(icao);  
    };
    
    // should never reach here
    return random() % (1U << 31);
}

TEST(KeyValidation, key_validation) {
    uint32_t test_key = create_random_key(AP_LocationDB::KeyDomain::MAVLINK);
    EXPECT_TRUE(AP_LocationDB::is_valid_key(test_key));

    test_key = create_random_key(AP_LocationDB::KeyDomain::ADSB);
    EXPECT_TRUE(AP_LocationDB::is_valid_key(test_key));

    test_key = create_random_key((AP_LocationDB::KeyDomain)((random() % (1U << 7)) + 10U));
    EXPECT_FALSE(AP_LocationDB::is_valid_key(test_key));

    test_key = create_random_key((AP_LocationDB::KeyDomain)(AP_LocationDB::KeyDomain::MAVLINK));
    // setting message id field to 255
    // remember key construction shortens the msg id
    // and 255 is the shortened message id for an unsupported mavlink message in Location DB 
    test_key |= 255U;
    EXPECT_FALSE(AP_LocationDB::is_valid_key(test_key));
}

TEST(KeyConstruction, key_construction) {
    uint8_t msgid = 144;
    uint8_t compid = 197;
    uint8_t sysid = 24;
    uint32_t key = AP_LocationDB::construct_key_mavlink(sysid, compid, msgid);

    EXPECT_EQ((AP_LocationDB::KeyDomain)((key >> 24) & 255U), AP_LocationDB::KeyDomain::MAVLINK);
    EXPECT_EQ((key >> 16) & 255U, sysid);
    EXPECT_EQ((key >> 8) & 255U, compid);
    EXPECT_EQ((key) & 255U, AP_LocationDB::short_mav_msg_id(msgid));

    uint32_t icao = (1U << 27) - 53U;
    key = AP_LocationDB::construct_key_adsb(icao);
    EXPECT_EQ((AP_LocationDB::KeyDomain)((key >> 24) & 255U), AP_LocationDB::KeyDomain::ADSB);
    EXPECT_NE(((1U << 24) - 1U) & key, icao);   // icao number must be of 24 bits. Bits higher than the 24th bit are zeroed

    icao =  (1U << 24) - 144U;
    key = AP_LocationDB::construct_key_adsb(icao);
    EXPECT_EQ((AP_LocationDB::KeyDomain)((key >> 24) & 255U), AP_LocationDB::KeyDomain::ADSB);
    EXPECT_EQ(((1U << 24) - 1U) & key, icao);
}

AP_LocationDB::DBItem create_random_db_item(uint8_t fields_to_populate) {
    AP_LocationDB::KeyDomain key_domains[] = { AP_LocationDB::KeyDomain::MAVLINK,
                                               AP_LocationDB::KeyDomain::ADSB};
    uint32_t timestamp = AP_HAL::millis();
    uint32_t item_key = create_random_key(key_domains[random() % (sizeof(key_domains)/sizeof(key_domains[0]))]);
    Vector3f pos {rand_float(), rand_float(), rand_float()};
    Vector3f vel {rand_float(), rand_float(), rand_float()};
    Vector3f acc {rand_float(), rand_float(), rand_float()};
    float heading = rand_float();
    float radius = rand_float();
    AP_LocationDB::DBItem item(item_key, timestamp, pos, vel, acc, heading, radius, fields_to_populate);

    return item;
}

AP_LocationDB::DBItem create_unique_db_item(uint8_t fields_to_populate, AP_LocationDB &db) {
    AP_LocationDB::DBItem item = create_random_db_item(fields_to_populate), dummy_item;
    
    while(db.get_item(item.get_key(), dummy_item)) {
        item = create_random_db_item(fields_to_populate);
    }
    return item;
}

TEST(ItemFields, item_fields) {
    for (uint8_t i = 0; i < (1 << 5); i++) {
        AP_LocationDB::DBItem item = create_random_db_item(i);
        Vector3f dummy_vector3f;
        float dummy_float;

        // Checking if we are able to retrive a populated field and vice versa
        // Using XOR (^) operator to match the received output of getters with the expected output
        // XOR returns false when both the operands have same value, i.e., either both of them are true or both are false
        EXPECT_FALSE(item.get_pos_NEU(dummy_vector3f) ^ ((i & (uint8_t)AP_LocationDB::DBItem::DataField::POS) != 0));
        EXPECT_FALSE(item.get_vel_NEU(dummy_vector3f) ^ ((i & (uint8_t)AP_LocationDB::DBItem::DataField::VEL) != 0));
        EXPECT_FALSE(item.get_acc_NEU(dummy_vector3f) ^ ((i & (uint8_t)AP_LocationDB::DBItem::DataField::ACC) != 0));
        EXPECT_FALSE(item.get_heading(dummy_float) ^ ((i & (uint8_t)AP_LocationDB::DBItem::DataField::HEADING) != 0));
        EXPECT_FALSE(item.get_radius(dummy_float) ^ ((i & (uint8_t)AP_LocationDB::DBItem::DataField::RADIUS) != 0));
    }
}

static void check_equal(AP_LocationDB::DBItem& item1, AP_LocationDB::DBItem& item2) {
    uint32_t timestamp_ms1 = item1.get_timestamp_ms(), timestamp_ms2 = item2.get_timestamp_ms();
    Vector3f pos1, pos2;
    Vector3f vel1, vel2;
    Vector3f acc1, acc2;
    float heading1, heading2;
    float radius1, radius2;

    EXPECT_TRUE(item1.get_pos_NEU(pos1));
    EXPECT_TRUE(item1.get_vel_NEU(vel1));
    EXPECT_TRUE(item1.get_acc_NEU(acc1));
    EXPECT_TRUE(item1.get_heading(heading1));
    EXPECT_TRUE(item1.get_radius(radius1));
    
    EXPECT_TRUE(item2.get_pos_NEU(pos2));
    EXPECT_TRUE(item2.get_vel_NEU(vel2));
    EXPECT_TRUE(item1.get_acc_NEU(acc2));
    EXPECT_TRUE(item2.get_heading(heading2));
    EXPECT_TRUE(item2.get_radius(radius2));
    
    EXPECT_EQ(timestamp_ms1, timestamp_ms2);
    EXPECT_EQ(pos1, pos2);
    EXPECT_EQ(vel1, vel2);
    EXPECT_EQ(acc1, acc2);
    EXPECT_EQ(heading1, heading2);
    EXPECT_EQ(radius1, radius2);

    return;
}

AP_LocationDB db;

TEST(Insertion, insert) {
    db.init();
    EXPECT_TRUE(db.healthy());
    EXPECT_TRUE(db.size() == 0);

    AP_LocationDB::DBItem arr[db.capacity()];

    for (int i=0; i<db.capacity(); i++) {
        AP_LocationDB::DBItem item = create_unique_db_item((1 << 5) - 1, db); // random unique data item with all fields populated
        arr[i] = item;
    }

    for (int i=0; i<db.capacity(); i++) {
        db.add_item(arr[i]);
    }

    EXPECT_TRUE(db.healthy());
    EXPECT_TRUE(db.size() == db.capacity());
    EXPECT_TRUE(db.is_full());

    // check consistency
    for (int i=0; i<db.capacity(); i++) {
        AP_LocationDB::DBItem item;
        EXPECT_TRUE(db.get_item(arr[i].get_key(), item));
        check_equal(item, arr[i]);
    }
}

TEST(Deletion, deletion) {
    db.init();
    EXPECT_TRUE(db.healthy());
    EXPECT_TRUE(db.size() == 0);

    int item_count = random() % db.capacity(); // push random no. of database items
    uint32_t keys[item_count];

    for (int i=0; i<item_count; i++) {
        AP_LocationDB::DBItem item = create_unique_db_item((1 << 5) - 1, db); // random unique data item with all fields populated
        keys[i] = item.get_key();
        EXPECT_TRUE(db.add_item(item));
    }

    EXPECT_TRUE(db.size() == item_count);

    // delete elements in random order
    while(item_count > 0) {
        uint16_t index = random() % item_count;
        EXPECT_TRUE(db.remove_item(keys[index]));
        keys[index] = keys[item_count - 1];
        EXPECT_TRUE(db.healthy());
        //EXPECT_TRUE(db.size() == item_count);
        item_count -= 1;
   }

   //EXPECT_TRUE(db.size() == 0);
}

TEST(Updation, updation) {
    db.init();
    EXPECT_TRUE(db.healthy());
    EXPECT_TRUE(db.size() == 0);

    int item_count = random() % db.capacity(); // push random no. of database items
    uint32_t keys[item_count];

    for (int i=0; i<item_count; i++) {
        AP_LocationDB::DBItem item = create_unique_db_item((1 << 5) - 1, db); // random unique data item with all fields populated
        keys[i] = item.get_key();
        EXPECT_TRUE(db.add_item(item));
    }

    for (int i=0; i<100; i++) {
        AP_LocationDB::DBItem new_item = create_unique_db_item((1 << 5) - 1, db);
        int update_index = random() % db.size();
        EXPECT_TRUE(db.update_item(keys[update_index], new_item));
        keys[update_index] = new_item.get_key();

        // check consistency
        AP_LocationDB::DBItem retrieved_item;
        EXPECT_TRUE(db.get_item(keys[update_index], retrieved_item));
        check_equal(new_item, retrieved_item);
    }
}

AP_GTEST_MAIN()

#endif // HAL_SITL or HAL_LINUX
