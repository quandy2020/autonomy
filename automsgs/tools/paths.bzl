"""Map proto/ paths onto the automsgs/ import layout used by protoc."""

def _stem(proto):
    if not proto.startswith("proto/") or not proto.endswith(".proto"):
        fail("unexpected proto path: %s" % proto)
    return proto[len("proto/"):-len(".proto")]

def _split(proto):
    stem = _stem(proto)
    directory, sep, name = stem.rpartition("/")
    if sep == "":
        fail("proto is not in a subdirectory: %s" % proto)
    return directory, name

def cpp_header(proto):
    return "automsgs/" + _stem(proto) + ".pb.h"

def cpp_source(proto):
    return "automsgs/" + _stem(proto) + ".pb.cc"

def detail_header(proto):
    directory, name = _split(proto)
    return "automsgs/%s/details/%s.pb.h" % (directory, name)

def python_module(proto):
    directory, name = _split(proto)
    return "python/automsgs/%s/%s_pb2.py" % (directory, name)
