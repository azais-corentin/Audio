#pragma once

#include "sweptsine.hh"
#include "synchronizedsweptsine.hh"

#include <variant>

namespace Audio::Generator {

using Base = std::variant<Null, SweptSine, SynchronizedSweptSine>;

}
