#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void DartLoader(pybind11::module& m)
{
  ::pybind11::class_<dart::utils::DartLoader>(m, "DartLoader")
      .def(::pybind11::init<>())
      .def(
          "addPackageDirectory",
          +[](dart::utils::DartLoader* self,
              const std::string& _packageName,
              const std::string& _packageDirectory) -> void {
            return self->addPackageDirectory(_packageName, _packageDirectory);
          },
          ::pybind11::arg("packageName"),
          ::pybind11::arg("packageDirectory"))
      .def(
          "parseSkeleton",
          +[](dart::utils::DartLoader* self,
              const dart::common::Uri& _uri) -> dart::dynamics::SkeletonPtr {
            return self->parseSkeleton(_uri);
          },
          ::pybind11::arg("uri"))
      .def(
          "parseSkeleton",
          +[](dart::utils::DartLoader* self,
              const dart::common::Uri& _uri,
              const dart::common::ResourceRetrieverPtr& _resourceRetriever)
              -> dart::dynamics::SkeletonPtr {
            return self->parseSkeleton(_uri, _resourceRetriever);
          },
          ::pybind11::arg("uri"),
          ::pybind11::arg("resourceRetriever"))
      .def(
          "parseSkeletonString",
          +[](dart::utils::DartLoader* self,
              const std::string& _urdfString,
              const dart::common::Uri& _baseUri)
              -> dart::dynamics::SkeletonPtr {
            return self->parseSkeletonString(_urdfString, _baseUri);
          },
          ::pybind11::arg("urdfString"),
          ::pybind11::arg("baseUri"))
      .def(
          "parseSkeletonString",
          +[](dart::utils::DartLoader* self,
              const std::string& _urdfString,
              const dart::common::Uri& _baseUri,
              const dart::common::ResourceRetrieverPtr& _resourceRetriever)
              -> dart::dynamics::SkeletonPtr {
            return self->parseSkeletonString(
                _urdfString, _baseUri, _resourceRetriever);
          },
          ::pybind11::arg("urdfString"),
          ::pybind11::arg("baseUri"),
          ::pybind11::arg("resourceRetriever"))
      .def(
          "parseWorld",
          +[](dart::utils::DartLoader* self, const dart::common::Uri& _uri)
              -> dart::simulation::WorldPtr { return self->parseWorld(_uri); },
          ::pybind11::arg("uri"))
      .def(
          "parseWorld",
          +[](dart::utils::DartLoader* self,
              const dart::common::Uri& _uri,
              const dart::common::ResourceRetrieverPtr& _resourceRetriever)
              -> dart::simulation::WorldPtr {
            return self->parseWorld(_uri, _resourceRetriever);
          },
          ::pybind11::arg("uri"),
          ::pybind11::arg("resourceRetriever"))
      .def(
          "parseWorldString",
          +[](dart::utils::DartLoader* self,
              const std::string& _urdfString,
              const dart::common::Uri& _baseUri) -> dart::simulation::WorldPtr {
            return self->parseWorldString(_urdfString, _baseUri);
          },
          ::pybind11::arg("urdfString"),
          ::pybind11::arg("baseUri"))
      .def(
          "parseWorldString",
          +[](dart::utils::DartLoader* self,
              const std::string& _urdfString,
              const dart::common::Uri& _baseUri,
              const dart::common::ResourceRetrieverPtr& _resourceRetriever)
              -> dart::simulation::WorldPtr {
            return self->parseWorldString(
                _urdfString, _baseUri, _resourceRetriever);
          },
          ::pybind11::arg("urdfString"),
          ::pybind11::arg("baseUri"),
          ::pybind11::arg("resourceRetriever"));
}

} // namespace python
} // namespace dart
