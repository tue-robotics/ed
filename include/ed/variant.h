#ifndef ED_VARIANT_H_
#define ED_VARIANT_H_

// Directly taken from http://stackoverflow.com/questions/5319216/implementing-a-variant-class

#include <boost/shared_ptr.hpp>

#include <string>

namespace ed
{

template <typename T> struct TypeWrapper
{
    using TYPE = T;
    using CONSTTYPE = T;
    using REFTYPE = T&;
    using CONSTREFTYPE = T&;
};

template <typename T> struct TypeWrapper<const T>
{
    using TYPE = T;
    using CONSTTYPE = T;
    using REFTYPE = T&;
    using CONSTREFTYPE = T&;
};

template <typename T> struct TypeWrapper<const T&>
{
    using TYPE = T;
    using CONSTTYPE = T;
    using REFTYPE = T&;
    using CONSTREFTYPE = T&;
};

template <typename T> struct TypeWrapper<T&>
{
    using TYPE = T;
    using CONSTTYPE = T;
    using REFTYPE = T&;
    using CONSTREFTYPE = T&;
};

class Variant
{
public:
    Variant() = default;

    template <class T> Variant(const T& inValue) : mImpl(new VariantImpl<typename TypeWrapper<T>::TYPE>(inValue)) {}

    template <class T> typename TypeWrapper<T>::REFTYPE getValue()
    {
        return dynamic_cast<VariantImpl<typename TypeWrapper<T>::TYPE>&>(*mImpl).mValue;
    }

    template <class T>
    [[nodiscard]] [[nodiscard]]
    typename TypeWrapper<T>::CONSTREFTYPE getValue() const
    {
        return dynamic_cast<VariantImpl<typename TypeWrapper<T>::TYPE>&>(*mImpl).mValue;
    }

    template <class T> void setValue(typename TypeWrapper<T>::CONSTREFTYPE inValue)
    {
        mImpl.reset(new VariantImpl<typename TypeWrapper<T>::TYPE>(inValue));
    }

private:
    struct AbstractVariantImpl
    {
        virtual ~AbstractVariantImpl() = default;
    };

    template <class T> struct VariantImpl : public AbstractVariantImpl
    {
        VariantImpl(const T& inValue) : mValue(inValue) {}

        ~VariantImpl() override = default;

        T mValue;
    };

    boost::shared_ptr<AbstractVariantImpl> mImpl;
};

} // namespace ed

#endif
