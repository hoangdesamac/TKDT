#pragma once
struct MockWire { void setClock(unsigned long) {} };
inline MockWire Wire;
