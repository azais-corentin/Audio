cmake_minimum_required(VERSION 3.30)

target_sources(Audio PRIVATE
    generators.hh
    abstractgenerator.hh
    abstractgenerator.cc
    sweptsine.hh
    sweptsine.cc
    synchronizedsweptsine.hh
    synchronizedsweptsine.cc
)
