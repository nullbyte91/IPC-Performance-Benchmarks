// This file is generated by omniidl (C++ backend)- omniORB_4_3. Do not edit.
#ifndef __ImageTransfer_hh__
#define __ImageTransfer_hh__

#ifndef __CORBA_H_EXTERNAL_GUARD__
#include <omniORB4/CORBA.h>
#endif

#ifndef  USE_stub_in_nt_dll
# define USE_stub_in_nt_dll_NOT_DEFINED_ImageTransfer
#endif
#ifndef  USE_core_stub_in_nt_dll
# define USE_core_stub_in_nt_dll_NOT_DEFINED_ImageTransfer
#endif
#ifndef  USE_dyn_stub_in_nt_dll
# define USE_dyn_stub_in_nt_dll_NOT_DEFINED_ImageTransfer
#endif






#ifdef USE_stub_in_nt_dll
# ifndef USE_core_stub_in_nt_dll
#  define USE_core_stub_in_nt_dll
# endif
# ifndef USE_dyn_stub_in_nt_dll
#  define USE_dyn_stub_in_nt_dll
# endif
#endif

#ifdef _core_attr
# error "A local CPP macro _core_attr has already been defined."
#else
# ifdef  USE_core_stub_in_nt_dll
#  define _core_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _core_attr
# endif
#endif

#ifdef _dyn_attr
# error "A local CPP macro _dyn_attr has already been defined."
#else
# ifdef  USE_dyn_stub_in_nt_dll
#  define _dyn_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _dyn_attr
# endif
#endif



_CORBA_MODULE ImageTransfer

_CORBA_MODULE_BEG

#ifndef __ImageTransfer_mImageService__
#define __ImageTransfer_mImageService__
  class ImageService;
  class _objref_ImageService;
  class _impl_ImageService;
  
  typedef _objref_ImageService* ImageService_ptr;
  typedef ImageService_ptr ImageServiceRef;

  class ImageService_Helper {
  public:
    typedef ImageService_ptr _ptr_type;

    static _ptr_type _nil();
    static _CORBA_Boolean is_nil(_ptr_type);
    static void release(_ptr_type);
    static void duplicate(_ptr_type);
    static void marshalObjRef(_ptr_type, cdrStream&);
    static _ptr_type unmarshalObjRef(cdrStream&);
  };

  typedef _CORBA_ObjRef_Var<_objref_ImageService, ImageService_Helper> ImageService_var;
  typedef _CORBA_ObjRef_OUT_arg<_objref_ImageService,ImageService_Helper > ImageService_out;

#endif

  // interface ImageService
  class ImageService {
  public:
    // Declarations for this interface type.
    typedef ImageService_ptr _ptr_type;
    typedef ImageService_var _var_type;

    static _ptr_type _duplicate(_ptr_type);
    static _ptr_type _narrow(::CORBA::Object_ptr);
    static _ptr_type _unchecked_narrow(::CORBA::Object_ptr);
    
    static _ptr_type _nil();

    static inline void _marshalObjRef(_ptr_type, cdrStream&);

    static inline _ptr_type _unmarshalObjRef(cdrStream& s) {
      omniObjRef* o = omniObjRef::_unMarshal(_PD_repoId,s);
      if (o)
        return (_ptr_type) o->_ptrToObjRef(_PD_repoId);
      else
        return _nil();
    }

    static inline _ptr_type _fromObjRef(omniObjRef* o) {
      if (o)
        return (_ptr_type) o->_ptrToObjRef(_PD_repoId);
      else
        return _nil();
    }

    static _core_attr const char* _PD_repoId;

    // Other IDL defined within this scope.
    
  };

  class _objref_ImageService :
    public virtual ::CORBA::Object,
    public virtual omniObjRef
  {
  public:
    // IDL operations
    ::CORBA::Boolean send_image(const char* image_data, ::CORBA::Long width, ::CORBA::Long height, ::CORBA::Long type);

    // Constructors
    inline _objref_ImageService()  { _PR_setobj(0); }  // nil
    _objref_ImageService(omniIOR*, omniIdentity*);

  protected:
    virtual ~_objref_ImageService();

    
  private:
    virtual void* _ptrToObjRef(const char*);

    _objref_ImageService(const _objref_ImageService&);
    _objref_ImageService& operator = (const _objref_ImageService&);
    // not implemented

    friend class ImageService;
  };

  class _pof_ImageService : public _OMNI_NS(proxyObjectFactory) {
  public:
    inline _pof_ImageService() : _OMNI_NS(proxyObjectFactory)(ImageService::_PD_repoId) {}
    virtual ~_pof_ImageService();

    virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
    virtual _CORBA_Boolean is_a(const char*) const;
  };

  class _impl_ImageService :
    public virtual omniServant
  {
  public:
    virtual ~_impl_ImageService();

    virtual ::CORBA::Boolean send_image(const char* image_data, ::CORBA::Long width, ::CORBA::Long height, ::CORBA::Long type) = 0;
    
  public:  // Really protected, workaround for xlC
    virtual _CORBA_Boolean _dispatch(omniCallHandle&);

  private:
    virtual void* _ptrToInterface(const char*);
    virtual const char* _mostDerivedRepoId();
    
  };


_CORBA_MODULE_END



_CORBA_MODULE POA_ImageTransfer
_CORBA_MODULE_BEG

  class ImageService :
    public virtual ImageTransfer::_impl_ImageService,
    public virtual ::PortableServer::ServantBase
  {
  public:
    virtual ~ImageService();

    inline ::ImageTransfer::ImageService_ptr _this() {
      return (::ImageTransfer::ImageService_ptr) _do_this(::ImageTransfer::ImageService::_PD_repoId);
    }
  };

_CORBA_MODULE_END



_CORBA_MODULE OBV_ImageTransfer
_CORBA_MODULE_BEG

_CORBA_MODULE_END





#undef _core_attr
#undef _dyn_attr



inline void
ImageTransfer::ImageService::_marshalObjRef(::ImageTransfer::ImageService_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}



#ifdef   USE_stub_in_nt_dll_NOT_DEFINED_ImageTransfer
# undef  USE_stub_in_nt_dll
# undef  USE_stub_in_nt_dll_NOT_DEFINED_ImageTransfer
#endif
#ifdef   USE_core_stub_in_nt_dll_NOT_DEFINED_ImageTransfer
# undef  USE_core_stub_in_nt_dll
# undef  USE_core_stub_in_nt_dll_NOT_DEFINED_ImageTransfer
#endif
#ifdef   USE_dyn_stub_in_nt_dll_NOT_DEFINED_ImageTransfer
# undef  USE_dyn_stub_in_nt_dll
# undef  USE_dyn_stub_in_nt_dll_NOT_DEFINED_ImageTransfer
#endif

#endif  // __ImageTransfer_hh__

