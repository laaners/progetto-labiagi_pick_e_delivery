#pragma once
#include <iostream>
#include <string>

template <typename T>
void printSet(const std::string& name, const T& container){
  std::cerr << name << ": [ ";
  for (const auto& it: container)
    std::cerr << it << " ";
  std::cerr << "] " << std::endl;
}

template <typename T>
void printMap(const std::string& name, const T& container){
  std::cerr << name << ": [ ";
  for (const auto& it: container)
    std::cerr << it.first << " ";
  std::cerr << "] " << std::endl;
}
