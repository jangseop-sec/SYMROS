#pragma once

#include <string>
#include <cstring>

#include "strcmp_macro.hpp"

inline bool operator==(const std::string& lhs, const char* rhs) {

  bool ret = true;

  if (lhs.size() != strlen(rhs)) ret &= false;

  const char * c = rhs;
  const char * s = lhs.data();

  int len = strlen(c);

  switch(len) {
    case 0: ret = true;
    case 1: STRCMP_1(s, c, ret); break;
    case 2: STRCMP_2(s, c, ret); break;
    case 3: STRCMP_3(s, c, ret); break;
    case 4: STRCMP_4(s, c, ret); break;
    case 5: STRCMP_5(s, c, ret); break;
    case 6: STRCMP_6(s, c, ret); break;
    case 7: STRCMP_7(s, c, ret); break;
    case 8: STRCMP_8(s, c, ret); break;
    case 9: STRCMP_9(s, c, ret); break;
    case 10: STRCMP_10(s, c, ret); break;
    case 11: STRCMP_11(s, c, ret); break;
    case 12: STRCMP_12(s, c, ret); break;
    case 13: STRCMP_13(s, c, ret); break;
    case 14: STRCMP_14(s, c, ret); break;
    case 15: STRCMP_15(s, c, ret); break;
    case 16: STRCMP_16(s, c, ret); break;
    case 17: STRCMP_17(s, c, ret); break;
    case 18: STRCMP_18(s, c, ret); break;
    case 19: STRCMP_19(s, c, ret); break;
    case 20: STRCMP_20(s, c, ret); break;
    case 21: STRCMP_21(s, c, ret); break;
    case 22: STRCMP_22(s, c, ret); break;
    case 23: STRCMP_23(s, c, ret); break;
    case 24: STRCMP_24(s, c, ret); break;
    case 25: STRCMP_25(s, c, ret); break;
    case 26: STRCMP_26(s, c, ret); break;
    case 27: STRCMP_27(s, c, ret); break;
    case 28: STRCMP_28(s, c, ret); break;
    case 29: STRCMP_29(s, c, ret); break;
    case 30: STRCMP_30(s, c, ret); break;
    case 31: STRCMP_31(s, c, ret); break;
    case 32: STRCMP_32(s, c, ret); break;
    case 33: STRCMP_33(s, c, ret); break;
    case 34: STRCMP_34(s, c, ret); break;
    case 35: STRCMP_35(s, c, ret); break;
    case 36: STRCMP_36(s, c, ret); break;
    case 37: STRCMP_37(s, c, ret); break;
    case 38: STRCMP_38(s, c, ret); break;
    case 39: STRCMP_39(s, c, ret); break;
    case 40: STRCMP_40(s, c, ret); break;  
    case 41: STRCMP_41(s, c, ret); break;
    case 42: STRCMP_42(s, c, ret); break;
    case 43: STRCMP_43(s, c, ret); break;
    case 44: STRCMP_44(s, c, ret); break;
    case 45: STRCMP_45(s, c, ret); break;
    case 46: STRCMP_46(s, c, ret); break;
    case 47: STRCMP_47(s, c, ret); break;
    case 48: STRCMP_48(s, c, ret); break;
    case 49: STRCMP_49(s, c, ret); break;
    case 50: STRCMP_50(s, c, ret); break;
    case 51: STRCMP_51(s, c, ret); break;
    case 52: STRCMP_52(s, c, ret); break;
    case 53: STRCMP_53(s, c, ret); break;
    case 54: STRCMP_54(s, c, ret); break;
    case 55: STRCMP_55(s, c, ret); break;
    case 56: STRCMP_56(s, c, ret); break;
    case 57: STRCMP_57(s, c, ret); break;
    case 58: STRCMP_58(s, c, ret); break;
    case 59: STRCMP_59(s, c, ret); break;
    case 60: STRCMP_60(s, c, ret); break;
    case 61: STRCMP_61(s, c, ret); break;
    case 62: STRCMP_62(s, c, ret); break;
    case 63: STRCMP_63(s, c, ret); break;
    case 64: STRCMP_64(s, c, ret); break;
    default: ret = strcmp(s, c) == 0; 
  }

  return ret;
}

inline bool operator==(const std::string& lhs, const std::string& rhs) {
  return operator==(lhs, rhs.data());
}

inline bool operator!=(const std::string& lhs, const char * rhs) {
  return !operator==(lhs, rhs);
}

inline bool operator!=(const std::string & lhs, const std::string & rhs) {
  return !operator==(lhs, rhs);
}
