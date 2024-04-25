#include <cassert>
#include <iostream>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/nice_type_name.h"

/// NOTE: This is just a concept experiments. Separators are used to show how
/// this file would be split into smaller files. Also, the order of includes
/// (the order of declarations here) matter. Also, names are simply for
/// demonstration. The actual code won't have JointBase<T>, but MobodBase<T>,
/// etc.

/// NOTE: Since JointBase must know the type of the data variant to work with,
/// all joint data types must be defined before JointBase can be defined.
/// Therefore in practice we'll have a separate file for each joint data type
/// (joint_xx_data.h), probably a single file that includes all the specific
/// data header files (say joint_data.h includes all the joint_xx_data.h) and
/// then joint base will include joint_data.h.
/// For this example, all data types are declare in the order they appear.

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// File joint_one_data.h
////////////////////////////////////////////////////////////////////////////////

/// Data type used by JointOne's computations.
struct JointOneData {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JointOneData)
  JointOneData() = default;
  double data{0.0};
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// File joint_two_data.h
////////////////////////////////////////////////////////////////////////////////

/// Data type used by JointTwo's computations.
struct JointTwoData {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JointTwoData)
  JointTwoData() = default;
  std::string data;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// File joint_base.h
////////////////////////////////////////////////////////////////////////////////

/// JointBase works with a "generic" data variant. Specific types will be
/// dispatched to work with their specific data type.
using JointDataVariant = std::variant<JointOneData, JointTwoData>;

/// Virtual interface for all joint types.
class JointBase {
 public:
  virtual void PrintName() const = 0;
  virtual ~JointBase() = default;
  virtual JointDataVariant MakeData() const = 0;
  virtual void CalcData(JointDataVariant*) const = 0;
  virtual void PrintData(const JointDataVariant&) const = 0;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// File joint_impl.h
////////////////////////////////////////////////////////////////////////////////

/// Each joint type must declare compile time information in its traits.
template <typename JointType>
struct JointTraits {};

#define DECLARE_JOINT_TRAITS(JointType)  \
  using Traits = JointTraits<JointType>; \
  using JointData = typename Traits::JointData;

/// CRTP type that implements the JointBase interface. Code that is shared
/// across joints will live here, optimized via CRTP and traits to specfic joint
/// types. In other words, this CRTP help us generate code with a common
/// structure (though maybe with different types and sizes) throuout joint
/// types. Calls within the scope of functions in this class, can be inlined by
/// the compiler. Therfore calls from JointBase are only reserved for high level
/// loops (say the kernel in the real code) so that polymorphic calls must only
/// be resolved once per type.
template <class Derived>
class JointImpl : public JointBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JointImpl)

  DECLARE_JOINT_TRAITS(Derived)

  JointImpl() = default;

  // JointImpl() : Base(std::in_place_type<Derived>) {}

  /// Prints the NiceTypeName of this joint's type.
  /// @pre Derived::DoPrintName() is defined.
  void PrintName() const override {
    std::cout << "JointImpl<" << drake::NiceTypeName::Get<Derived>() << ">"
              << std::endl;
    /// Call type specific implementation. This call, and others if made here,
    /// can be inlined within this scope.
    derived().DoPrintName();
    std::cout << std::endl;
  }

  /// Makes data used by `Derived` to perform computations.
  /// @pre JointTraits<Derived>::JointData is default constructible.
  JointDataVariant MakeData() const override {
    static_assert(std::is_default_constructible<JointData>::value,
                  "Joint data must be default constructible");
    return JointDataVariant{std::in_place_type<JointData>};
  }

  /// Computes data for this joint.
  /// @pre Derived::DoCalcData() is defined.
  void CalcData(JointDataVariant* data_variant) const override {
    DRAKE_ASSERT(data_variant != nullptr);
    auto& data = std::get<JointData>(*data_variant);
    derived().DoCalcData(&data);
  }

  /// Prints joint type specific data.
  /// @pre Derived::DoPrintData() must be defined.
  void PrintData(const JointDataVariant& data_variant) const override {
    const auto& data = std::get<JointData>(data_variant);
    derived().DoPrintData(data);
  }

 protected:
  const Derived& derived() const { return *static_cast<const Derived*>(this); }
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// File joint_one.h
////////////////////////////////////////////////////////////////////////////////

class JointOne;  // Forward declaration needed to define traits.

template <>
struct JointTraits<JointOne> {
  using JointData = JointOneData;
};

/// A specific joint type.
class JointOne final : public JointImpl<JointOne> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JointOne)

  DECLARE_JOINT_TRAITS(JointOne)

  JointOne() {}

  JointOne(int data) : data_(data) {}

 private:
  // Give JointImpl access to private implementations.
  friend JointImpl<JointOne>;

  // Implementation of JointImpl::PrintName().
  void DoPrintName() const {
    std::cout << "JointOne with data = " << data_ << "\n";
  }

  // Implementation of JointImpl::CalcData().
  void DoCalcData(JointData* joint_data) const { joint_data->data = 3.14; }

  // Implementation of JointImpl::PrintData().
  void DoPrintData(const JointData& joint_data) const {
    std::cout << "JointOne data: " << joint_data.data << std::endl;
  }

  int data_;  // dummy data.
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// File joint_two.h
////////////////////////////////////////////////////////////////////////////////

class JointTwo;  // Forward declaration needed to define traits.

template <>
struct JointTraits<JointTwo> {
  using JointData = JointTwoData;
};

/// Another joint type, see notes in JointOne.
class JointTwo : public JointImpl<JointTwo> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JointTwo)

  DECLARE_JOINT_TRAITS(JointTwo)

  JointTwo() = default;

  JointTwo(double d) : data_(d) {}

 private:
  friend JointImpl<JointTwo>;

  // Implementation of JointImpl::PrintName().
  void DoPrintName() const {
    std::cout << "JointTwo with data = " << data_ << "\n";
  }

  // Implementation of JointImpl::CalcData().
  void DoCalcData(JointData* joint_data) const {
    joint_data->data = "hello joints";
  }

  // Implementation of JointImpl::PrintData().
  void DoPrintData(const JointData& joint_data) const {
    std::cout << "JointTwo data: " << joint_data.data << std::endl;
  }

  double data_;
  double data2_;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// File type_list.h
////////////////////////////////////////////////////////////////////////////////

template <typename... Ts>
struct SizeOfList;

template <template <typename...> class TL, typename... Ts>
struct SizeOfList<TL<Ts...>> {
  constexpr static auto value = sizeof...(Ts);
};

template <class... Ts>
constexpr size_t TypeListMaxSize() noexcept {
  size_t maxSize{0};
  ((maxSize = std::max(maxSize, sizeof(Ts))), ...);
  return maxSize;
}

// A structs that represents a compile-time list of types.
template <typename...>
struct TypeList;

// Compile-time size of a TypeList.
template <typename TList>
struct Length;

template <typename... Types>
struct Length<TypeList<Types...>> {
  static constexpr std::size_t value = sizeof...(Types);
};

// Retrieves the type T in TList with the largest sizeof(T).
template <typename TList>
struct LargestType;

// Specialization for a list with a single type.
template <typename T>
struct LargestType<TypeList<T>> {
  using type = T;
};

// Recursive specialization for lists with more than one type.
template <typename T, typename U, typename... Ts>
struct LargestType<TypeList<T, U, Ts...>> {
  using type = typename LargestType<
      TypeList<typename std::conditional<(sizeof(U) <= sizeof(T)), T, U>::type,
               Ts...>>::type;
};

// Verifies that each T in TList inherits from TBase.
template <typename TBase, typename TList>
struct IsBaseOf;

// Specialization for a list with a single type.
template <typename TBase, typename T>
struct IsBaseOf<TBase, TypeList<T>> {
  constexpr static bool value = std::is_base_of<TBase, T>::value;
};

// Recursive specialization for lists with more than one type.
template <typename TBase, typename T, typename U, typename... Ts>
struct IsBaseOf<TBase, TypeList<T, U, Ts...>> {
  constexpr static bool value = std::is_base_of<TBase, T>::value &&
                                IsBaseOf<TBase, TypeList<U, Ts...>>::value;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// File pool.h
////////////////////////////////////////////////////////////////////////////////

/// Pool is a container of objects with types that belong to a closed set of
/// types, specified by TList.
///
/// Objects in a pool are stored by value, contiguously in memory, in order to
/// minimize cache misses when traversing elements in the pool.
/// To accomodate objects with types of different sizes, each value is stored
/// in a bucket of size equal to the maximum type size in TList.
///
/// Objects in TList are polymorphic, with base class TBase, even when stored
/// contiguously in memory by value.
///
/// @tparam TList a TypeList that specifies the list of types. Types T in TList
/// must inherit from TBase.
/// @tparam TBase the base class of polymorphic objects in TList.
template <typename TList, typename TBase>
class Pool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Pool)

  /// Assert that elements in TList inherit from TBase.
  static_assert(IsBaseOf<TBase, TList>::value);

  /// Constructs a pool with a given `capacity`.
  /// After this call, we cannot place more than `capacity` objects in the pool.
  explicit Pool(int capacity) { pool_.reserve(capacity); }

  /// Constructs an object of type U directly in the pool, forwarding arguments
  /// `args`.
  ///
  /// A call to this method looks like:
  ///  pool.emplace_back<Type>(arg1, arg2, ...);
  ///
  /// @pre size() < capacity().
  /// @pre U must be a type in TList.
  /// @pre U must inherit from TBase.
  template <typename U, class... Args>
  void emplace_back(Args&&... args) {
    DRAKE_DEMAND(size() < capacity());
    static_assert(std::is_base_of_v<TBase, U>);
    // TODO: Some template code or SFINAE to bark if U is not one of the
    // supported types.
    pool_.push_back(Bucket{});  // un-initialized bucket, increases vector size.
    new (&pool_.back())
        U(std::forward<Args>(args)...);  // Placement new into the new bucket.
  }

  /// The maximum number of objects this pool can store.
  int capacity() const { return static_cast<int>(pool_.capacity()); }

  /// The current number of objects stored by the pool.
  int size() const { return ssize(pool_); }

  /// Returns a refernce to the common interface for the i-th object.
  const JointBase& operator[](int i) const {
    return *reinterpret_cast<const JointBase*>(&pool_[i]);
  }

 private:
  /// The pool stores equal sized buckets. The size is determined at
  /// compile-time as the maximum sizeof(T) with T in TList.
  struct Bucket {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Bucket)
    constexpr static int kBucketSize =
        sizeof(typename LargestType<TList>::type);
    Bucket() = default;
    std::byte storage_[kBucketSize];
  };
  std::vector<Bucket> pool_;  // The actual pool of objects.
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// Application file
////////////////////////////////////////////////////////////////////////////////

int main() {
  // Compile-time list of the closed set of joints we'll work with.
  using JointsList = TypeList<JointOne, JointTwo>;
  static_assert(IsBaseOf<JointBase, JointsList>::value);

  // Some experiments with compile-time information.
  std::cout << "List size: " << Length<JointsList>::value << std::endl;
  std::cout << "sizeof(JointOne): " << sizeof(JointOne) << std::endl;
  std::cout << "sizeof(JointTwo): " << sizeof(JointTwo) << std::endl;
  std::cout << "The largest type is: "
            << drake::NiceTypeName::Get<LargestType<JointsList>::type>()
            << std::endl;

  // Create a pool for three joints, push some joints, and print their names to
  // test the virtual dispatching is working properly.
  using JointPool = Pool<JointsList, JointBase>;
  JointPool pool(3);
  pool.emplace_back<JointTwo>();
  pool.emplace_back<JointOne>(1 /* data */);
  pool.emplace_back<JointTwo>(3.14 /* data */);
  for (int i = 0; i < pool.size(); ++i) {
    pool[i].PrintName();
  }

  // Make data.
  std::vector<JointDataVariant> data;
  data.reserve(pool.size());
  for (int i = 0; i < pool.size(); ++i) {
    data.push_back(pool[i].MakeData());
  }

  // Work with data.
  for (int i = 0; i < pool.size(); ++i) {
    pool[i].CalcData(&data[i]);
  }

  // Print the results.
  for (int i = 0; i < pool.size(); ++i) {
    pool[i].PrintData(data[i]);
  }
}
