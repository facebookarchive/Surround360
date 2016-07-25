/*
This is a fork of SuperEasyJSON, a small library for parsing JSON (which is MIT
licensed). Homepage here:
http://www.sourceforge.net/p/supereasyjson

Changes in fork:
- Cleaned up formatting so it can pass minimum fbcode landing requirements.
- Fixed a bug/issue where serializing/deserializing doubles could result in
    loss of precision.
 */

#pragma once

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

// PLEASE SEE THE README FOR USAGE INFORMATION AND EXAMPLES. Comments will be
// kept to a minimum to reduce clutter.
namespace json {

enum ValueType {
  NULLVal,
  StringVal,
  IntVal,
  FloatVal,
  DoubleVal,
  ObjectVal,
  ArrayVal,
  BoolVal
};

class Value;

// Represents a JSON object which is of the form {string:value, string:value,
// ...} Where string is the "key" name and is
// of the form "" or "characters". Value is either of: string, number, object,
// array, boolean, null
class Object {
public:
  // This is the type used to store key/value pairs. If you want to get an
  // iterator for this class to iterate over its members,
  // use this.
  // For example: Object::ValueMap::iterator my_iterator;
  typedef std::map<std::string, Value> ValueMap;

protected:
  ValueMap mValues;

public:
  Object();
  Object(const Object &obj);

  Object &operator=(const Object &obj);

  friend bool operator==(const Object &lhs, const Object &rhs);
  inline friend bool operator!=(const Object &lhs, const Object &rhs) {
    return !(lhs == rhs);
  }
  friend bool operator<(const Object &lhs, const Object &rhs);
  inline friend bool operator>(const Object &lhs, const Object &rhs) {
    return operator<(rhs, lhs);
  }
  inline friend bool operator<=(const Object &lhs, const Object &rhs) {
    return !operator>(lhs, rhs);
  }
  inline friend bool operator>=(const Object &lhs, const Object &rhs) {
    return !operator<(lhs, rhs);
  }

  // Just like a std::map, you can get the value for a key by using the index
  // operator. You could also
  // use this to insert a value if it doesn't exist, or overwrite it if it does.
  // Example:
  // Value my_val = my_object["some key name"];
  // my_object["some key name"] = "overwriting the value with this new string
  // value";
  // my_object["new key name"] = "a new key being inserted";
  Value &operator[](const std::string &key);
  const Value &operator[](const std::string &key) const;
  Value &operator[](const char *key);
  const Value &operator[](const char *key) const;

  ValueMap::const_iterator begin() const;
  ValueMap::const_iterator end() const;
  ValueMap::iterator begin();
  ValueMap::iterator end();

  // Find will return end() if the key can't be found, just like std::map does.
  // ->first will be the key (a std::string),
  // ->second will be the Value.
  ValueMap::iterator find(const std::string &key);
  ValueMap::const_iterator find(const std::string &key) const;

  // Convenience wrapper to search for a key
  bool HasKey(const std::string &key) const;

  // Checks if the object contains all the keys in the array. If it does,
  // returns -1.
  // If it doesn't, returns the index of the first key it couldn't find.
  int HasKeys(const std::vector<std::string> &keys) const;
  int HasKeys(const char *keys[], int key_count) const;

  // Removes all values and resets the state back to default
  void Clear();

  size_t size() const { return mValues.size(); }
};

// Represents a JSON Array which is of the form [value, value, ...] where value
// is either of: string, number, object, array, boolean, null
class Array {
public:
  // This is the type used to store values. If you want to get an iterator for
  // this class to iterate over its members,
  // use this.
  // For example: Array::ValueVector::iterator my_array_iterator;
  typedef std::vector<Value> ValueVector;

protected:
  ValueVector mValues;

public:
  Array();
  Array(const Array &a);

  Array &operator=(const Array &a);

  friend bool operator==(const Array &lhs, const Array &rhs);
  inline friend bool operator!=(const Array &lhs, const Array &rhs) {
    return !(lhs == rhs);
  }
  friend bool operator<(const Array &lhs, const Array &rhs);
  inline friend bool operator>(const Array &lhs, const Array &rhs) {
    return operator<(rhs, lhs);
  }
  inline friend bool operator<=(const Array &lhs, const Array &rhs) {
    return !operator>(lhs, rhs);
  }
  inline friend bool operator>=(const Array &lhs, const Array &rhs) {
    return !operator<(lhs, rhs);
  }

  Value &operator[](size_t i);
  const Value &operator[](size_t i) const;

  ValueVector::const_iterator begin() const;
  ValueVector::const_iterator end() const;
  ValueVector::iterator begin();
  ValueVector::iterator end();

  // Just a convenience wrapper for doing a std::find(Array::begin(),
  // Array::end(), Value)
  ValueVector::iterator find(const Value &v);
  ValueVector::const_iterator find(const Value &v) const;

  // Convenience wrapper to check if a value is in the array
  bool HasValue(const Value &v) const;

  // Removes all values and resets the state back to default
  void Clear();

  void push_back(const Value &v);
  void insert(size_t index, const Value &v);
  size_t size() const;
};

// Represents a JSON value which is either of: string, number, object, array,
// boolean, null
class Value {
protected:
  ValueType mValueType;
  int mIntVal;
  float mFloatVal;
  double mDoubleVal;
  std::string mStringVal;
  Object mObjectVal;
  Array mArrayVal;
  bool mBoolVal;

public:
  Value()
      : mValueType(NULLVal), mIntVal(0), mFloatVal(0), mDoubleVal(0),
        mBoolVal(false) {}
  Value(int v)
      : mValueType(IntVal), mIntVal(v), mFloatVal((float)v),
        mDoubleVal((double)v), mBoolVal(false) {}
  Value(float v)
      : mValueType(FloatVal), mIntVal((int)v), mFloatVal(v),
        mDoubleVal((double)v), mBoolVal(false) {}
  Value(double v)
      : mValueType(DoubleVal), mIntVal((int)v), mFloatVal((float)v),
        mDoubleVal(v), mBoolVal(false) {}
  Value(const std::string &v)
      : mValueType(StringVal), mIntVal(), mFloatVal(), mDoubleVal(),
        mStringVal(v), mBoolVal(false) {}
  Value(const char *v)
      : mValueType(StringVal), mIntVal(), mFloatVal(), mDoubleVal(),
        mStringVal(v), mBoolVal(false) {}
  Value(const Object &v)
      : mValueType(ObjectVal), mIntVal(), mFloatVal(), mDoubleVal(),
        mObjectVal(v), mBoolVal(false) {}
  Value(const Array &v)
      : mValueType(ArrayVal), mIntVal(), mFloatVal(), mDoubleVal(),
        mArrayVal(v), mBoolVal(false) {}
  Value(bool v)
      : mValueType(BoolVal), mIntVal(), mFloatVal(), mDoubleVal(), mBoolVal(v) {
  }
  Value(const Value &v);

  // Use this to determine the underlying type that this Value class represents.
  // It will be one of the
  // ValueType enums as defined at the top of this file.
  ValueType GetType() const { return mValueType; }

  // Convenience method that checks if this type is an int/double/float
  bool IsNumeric() const {
    return (mValueType == IntVal) || (mValueType == DoubleVal) ||
           (mValueType == FloatVal);
  }

  Value &operator=(const Value &v);

  friend bool operator==(const Value &lhs, const Value &rhs);
  inline friend bool operator!=(const Value &lhs, const Value &rhs) {
    return !(lhs == rhs);
  }
  friend bool operator<(const Value &lhs, const Value &rhs);
  inline friend bool operator>(const Value &lhs, const Value &rhs) {
    return operator<(rhs, lhs);
  }
  inline friend bool operator<=(const Value &lhs, const Value &rhs) {
    return !operator>(lhs, rhs);
  }
  inline friend bool operator>=(const Value &lhs, const Value &rhs) {
    return !operator<(lhs, rhs);
  }

  // If this value represents an object or array, you can use the [] indexing
  // operator
  // just like you would with the native json::Array or json::Object classes.
  // THROWS A std::runtime_error IF NOT AN ARRAY OR OBJECT.
  Value &operator[](size_t idx);
  const Value &operator[](size_t idx) const;
  Value &operator[](const std::string &key);
  const Value &operator[](const std::string &key) const;
  Value &operator[](const char *key);
  const Value &operator[](const char *key) const;

  // If this value represents an object, these methods let you check if a single
  // key or an array of
  // keys is contained within it.
  // THROWS A std::runtime_error IF NOT AN OBJECT.
  bool HasKey(const std::string &key) const;
  int HasKeys(const std::vector<std::string> &keys) const;
  int HasKeys(const char *keys[], int key_count) const;

  // non-operator versions, **will throw a std::runtime_error if invalid with an
  // appropriate error message**
  int ToInt() const;
  float ToFloat() const;
  double ToDouble() const;
  bool ToBool() const;
  const std::string &ToString() const;
  Object ToObject() const;
  Array ToArray() const;

  // These versions do the same as above but will return your specified default
  // value in the event there's an error, and thus **don't** throw an exception.
  int ToInt(int def) const { return IsNumeric() ? mIntVal : def; }
  float ToFloat(float def) const { return IsNumeric() ? mFloatVal : def; }
  double ToDouble(double def) const { return IsNumeric() ? mDoubleVal : def; }
  bool ToBool(bool def) const {
    return (mValueType == BoolVal) ? mBoolVal : def;
  }
  const std::string &ToString(const std::string &def) const {
    return (mValueType == StringVal) ? mStringVal : def;
  }

  // Please note that as per C++ rules, implicitly casting a Value to a
  // std::string won't work.
  // This is because it could use the int/float/double/bool operators as well.
  // So to assign a
  // Value to a std::string you can either do:
  // 		my_string = (std::string)my_value
  // Or you can now do:
  // 		my_string = my_value.ToString();
  //
  operator int() const;
  operator float() const;
  operator double() const;
  operator bool() const;
  operator std::string() const;
  operator Object() const;
  operator Array() const;

  // Returns 1 for anything not an Array/ObjectVal
  size_t size() const;

  // Resets the state back to default, aka NULLVal
  void Clear();
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Converts a JSON Object or Array instance into a JSON string representing it.
// RETURNS EMPTY STRING ON ERROR.
// As per JSON specification, a JSON data structure must be an array or an
// object. Thus, you must either pass in a
// json::Array, json::Object, or a json::Value that has an Array or Object as
// its underlying type.
std::string Serialize(const Value &obj);

// If there is an error, Value will be NULLVal. Pass in a valid JSON string
// (such as one returned from Serialize, or obtained
// elsewhere) to receive a Value in return that represents the JSON structure.
// Check the type of Value by calling GetType().
// It will be ObjectVal or ArrayVal (or NULLVal if invalid JSON). The Value
// class contains the operator [] for indexing in the
// case that the underlying type is an object or array. You may, if you prefer,
// create an object or array from the Value returned
// by this method by simply passing it into the constructor.
Value Deserialize(const std::string &str);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
inline bool operator==(const Object &lhs, const Object &rhs) {
  return lhs.mValues == rhs.mValues;
}

inline bool operator<(const Object &lhs, const Object &rhs) {
  return lhs.mValues < rhs.mValues;
}

inline bool operator==(const Array &lhs, const Array &rhs) {
  return lhs.mValues == rhs.mValues;
}

inline bool operator<(const Array &lhs, const Array &rhs) {
  return lhs.mValues < rhs.mValues;
}

/* When comparing different numeric types, this method works the same as if you
 compared different numeric types
 on your own. Thus it performs the same as if you, for example, did this:

        int a = 1;
        float b = 1.1f;
        bool equivalent = a == b;

        The same logic applies to the other comparison operators.
 */
inline bool operator==(const Value &lhs, const Value &rhs) {
  if ((lhs.mValueType != rhs.mValueType) && !lhs.IsNumeric() &&
      !rhs.IsNumeric())
    return false;

  switch (lhs.mValueType) {
  case StringVal:
    return lhs.mStringVal == rhs.mStringVal;

  case IntVal:
    if (rhs.GetType() == FloatVal)
      return lhs.mIntVal == rhs.mFloatVal;
    else if (rhs.GetType() == DoubleVal)
      return lhs.mIntVal == rhs.mDoubleVal;
    else if (rhs.GetType() == IntVal)
      return lhs.mIntVal == rhs.mIntVal;
    else
      return false;

  case FloatVal:
    if (rhs.GetType() == FloatVal)
      return lhs.mFloatVal == rhs.mFloatVal;
    else if (rhs.GetType() == DoubleVal)
      return lhs.mFloatVal == rhs.mDoubleVal;
    else if (rhs.GetType() == IntVal)
      return lhs.mFloatVal == rhs.mIntVal;
    else
      return false;

  case DoubleVal:
    if (rhs.GetType() == FloatVal)
      return lhs.mDoubleVal == rhs.mFloatVal;
    else if (rhs.GetType() == DoubleVal)
      return lhs.mDoubleVal == rhs.mDoubleVal;
    else if (rhs.GetType() == IntVal)
      return lhs.mDoubleVal == rhs.mIntVal;
    else
      return false;

  case BoolVal:
    return lhs.mBoolVal == rhs.mBoolVal;

  case ObjectVal:
    return lhs.mObjectVal == rhs.mObjectVal;

  case ArrayVal:
    return lhs.mArrayVal == rhs.mArrayVal;

  default:
    return true;
  }
}

inline bool operator<(const Value &lhs, const Value &rhs) {
  if ((lhs.mValueType != rhs.mValueType) && !lhs.IsNumeric() &&
      !rhs.IsNumeric())
    return false;

  switch (lhs.mValueType) {
  case StringVal:
    return lhs.mStringVal < rhs.mStringVal;

  case IntVal:
    if (rhs.GetType() == FloatVal)
      return lhs.mIntVal < rhs.mFloatVal;
    else if (rhs.GetType() == DoubleVal)
      return lhs.mIntVal < rhs.mDoubleVal;
    else if (rhs.GetType() == IntVal)
      return lhs.mIntVal < rhs.mIntVal;
    else
      return false;

  case FloatVal:
    if (rhs.GetType() == FloatVal)
      return lhs.mFloatVal < rhs.mFloatVal;
    else if (rhs.GetType() == DoubleVal)
      return lhs.mFloatVal < rhs.mDoubleVal;
    else if (rhs.GetType() == IntVal)
      return lhs.mFloatVal < rhs.mIntVal;
    else
      return false;

  case DoubleVal:
    if (rhs.GetType() == FloatVal)
      return lhs.mDoubleVal < rhs.mFloatVal;
    else if (rhs.GetType() == DoubleVal)
      return lhs.mDoubleVal < rhs.mDoubleVal;
    else if (rhs.GetType() == IntVal)
      return lhs.mDoubleVal < rhs.mIntVal;
    else
      return false;

  case BoolVal:
    return lhs.mBoolVal < rhs.mBoolVal;

  case ObjectVal:
    return lhs.mObjectVal < rhs.mObjectVal;

  case ArrayVal:
    return lhs.mArrayVal < rhs.mArrayVal;

  default:
    return true;
  }
}

}
