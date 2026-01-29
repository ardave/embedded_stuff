#include "unity.h"
#include "nmea_parser.h"

void setUp(void) {
}

void tearDown(void) {
}

void test_valid_gga_sentence(void) {
    /* Real GGA sentence with fix */
    const char *sentence = "$GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*42";
    gps_data_t data;

    TEST_ASSERT_EQUAL_INT(0, nmea_parse_gga(sentence, &data));
    TEST_ASSERT_EQUAL_UINT8(12, data.hours);
    TEST_ASSERT_EQUAL_UINT8(35, data.minutes);
    TEST_ASSERT_EQUAL_UINT8(19, data.seconds);
    TEST_ASSERT_EQUAL_UINT8(1, data.fix_quality);
    TEST_ASSERT_EQUAL_UINT8(8, data.satellites);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 48.1173f, data.latitude);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 11.51667f, data.longitude);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.9f, data.hdop);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 545.4f, data.altitude);
}

void test_empty_fields(void) {
    /* GGA with empty lat/lon fields (no fix) */
    const char *sentence = "$GNGGA,123519.00,,,,,0,00,,,,,,,*56";
    gps_data_t data;

    TEST_ASSERT_EQUAL_INT(0, nmea_parse_gga(sentence, &data));
    TEST_ASSERT_EQUAL_UINT8(12, data.hours);
    TEST_ASSERT_EQUAL_UINT8(35, data.minutes);
    TEST_ASSERT_EQUAL_UINT8(19, data.seconds);
    TEST_ASSERT_EQUAL_UINT8(0, data.fix_quality);
    TEST_ASSERT_EQUAL_UINT8(0, data.satellites);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, data.latitude);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, data.longitude);
}

void test_wrong_sentence_type(void) {
    /* RMC sentence should be rejected */
    const char *sentence = "$GNRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
    gps_data_t data;

    TEST_ASSERT_EQUAL_INT(-1, nmea_parse_gga(sentence, &data));
}

void test_south_latitude_west_longitude(void) {
    /* Southern hemisphere, western hemisphere */
    const char *sentence = "$GNGGA,123519.00,3356.000,S,11803.000,W,1,04,1.5,100.0,M,0.0,M,,*00";
    gps_data_t data;

    TEST_ASSERT_EQUAL_INT(0, nmea_parse_gga(sentence, &data));
    TEST_ASSERT_TRUE(data.latitude < 0);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -33.9333f, data.latitude);
    TEST_ASSERT_TRUE(data.longitude < 0);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -118.05f, data.longitude);
}

void test_single_digit_satellites(void) {
    /* Single digit satellite count */
    const char *sentence = "$GNGGA,123519.00,4807.038,N,01131.000,E,1,3,0.9,545.4,M,47.0,M,,*42";
    gps_data_t data;

    TEST_ASSERT_EQUAL_INT(0, nmea_parse_gga(sentence, &data));
    TEST_ASSERT_EQUAL_UINT8(3, data.satellites);
}

void test_double_digit_satellites(void) {
    /* Double digit satellite count */
    const char *sentence = "$GNGGA,123519.00,4807.038,N,01131.000,E,1,12,0.9,545.4,M,47.0,M,,*42";
    gps_data_t data;

    TEST_ASSERT_EQUAL_INT(0, nmea_parse_gga(sentence, &data));
    TEST_ASSERT_EQUAL_UINT8(12, data.satellites);
}

void test_null_sentence(void) {
    gps_data_t data;
    TEST_ASSERT_EQUAL_INT(-1, nmea_parse_gga(NULL, &data));
}

void test_null_data(void) {
    const char *sentence = "$GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*42";
    TEST_ASSERT_EQUAL_INT(-1, nmea_parse_gga(sentence, NULL));
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_valid_gga_sentence);
    RUN_TEST(test_empty_fields);
    RUN_TEST(test_wrong_sentence_type);
    RUN_TEST(test_south_latitude_west_longitude);
    RUN_TEST(test_single_digit_satellites);
    RUN_TEST(test_double_digit_satellites);
    RUN_TEST(test_null_sentence);
    RUN_TEST(test_null_data);

    return UNITY_END();
}
