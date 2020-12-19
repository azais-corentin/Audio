#include "baseinterface.hh"

namespace Audio::Generator {

Null::Null() : BaseInterface<Null>(0, 0) {}

float Null::sample(size_t) const { return 0; }

} // namespace Audio::Generator
