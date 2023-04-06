#pragma once

#include "msgpack.hpp"

namespace msgpack {
  MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {
    namespace adaptor {

      template<>
      struct convert<PointCloudData> {
        msgpack::object const& operator()(msgpack::object const& o, PointCloudData& v) const {
          if (o.type != msgpack::type::ARRAY) {
            throw msgpack::type_error();
          }

          msgpack::object_array arr = o.via.array;
          if (arr.size != 4) {
            throw msgpack::type_error();
          }

          v.points = arr.ptr[0].as<std::vector<float>>();
          v.width = arr.ptr[1].as<uint32_t>();
          v.height = arr.ptr[2].as<uint32_t>();
          v.is_dense = arr.ptr[3].as<bool>();

          return o;
        }
      };

      template<>
      struct pack<PointCloudData> {
        template <typename Stream>
        packer<Stream>& operator()(msgpack::packer<Stream>& o, PointCloudData const& v) const {
          o.pack_array(4);
          o.pack(v.points);
          o.pack(v.width);
          o.pack(v.height);
          o.pack(v.is_dense);
          return o;
        }
      };

    } // namespace adaptor
  } // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
} // namespace msgpack
