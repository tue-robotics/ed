#ifndef ED_VARIANT_H_
#define ED_VARIANT_H_

// Directly taken from http://stackoverflow.com/questions/5319216/implementing-a-variant-class

#include <boost/shared_ptr.hpp>

#include <string>

namespace ed
{

template <typename T>
struct TypeWrapper
{
    typedef T TYPE;
    typedef const T CONSTTYPE;
    typedef T& REFTYPE;
    typedef const T& CONSTREFTYPE;
};

template <typename T>
struct TypeWrapper<const T>
{
    typedef T TYPE;
    typedef const T CONSTTYPE;
    typedef T& REFTYPE;
    typedef const T& CONSTREFTYPE;
};

template <typename T>
struct TypeWrapper<const T&>
{
    typedef T TYPE;
    typedef const T CONSTTYPE;
    typedef T& REFTYPE;
    typedef const T& CONSTREFTYPE;
};

template <typename T>
struct TypeWrapper<T&>
{
    typedef T TYPE;
    typedef const T CONSTTYPE;
    typedef T& REFTYPE;
    typedef const T& CONSTREFTYPE;
};

class Variant
{
public:
    Variant() { }

    template<class T>
    Variant(T inValue) :
        mImpl(new VariantImpl<typename TypeWrapper<T>::TYPE>(inValue))
    {
    }

    template<class T>
    typename TypeWrapper<T>::REFTYPE getValue()
    {
        return dynamic_cast<VariantImpl<typename TypeWrapper<T>::TYPE>&>(*mImpl.get()).mValue;
    }

    template<class T>
    typename TypeWrapper<T>::CONSTREFTYPE getValue() const
    {
        return dynamic_cast<VariantImpl<typename TypeWrapper<T>::TYPE>&>(*mImpl.get()).mValue;
    }

    template<class T>
    void setValue(typename TypeWrapper<T>::CONSTREFTYPE inValue)
    {
        mImpl.reset(new VariantImpl<typename TypeWrapper<T>::TYPE>(inValue));
    }

private:
    struct AbstractVariantImpl
    {
        virtual ~AbstractVariantImpl() {}
    };

    template<class T>
    struct VariantImpl : public AbstractVariantImpl
    {
        VariantImpl(T inValue) : mValue(inValue) { }

        ~VariantImpl() {}

        T mValue;
    };

    boost::shared_ptr<AbstractVariantImpl> mImpl;
};

} // end namespace

#endif
