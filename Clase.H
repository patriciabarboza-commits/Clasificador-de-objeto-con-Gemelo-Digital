#pragma once


class NombreClase {
private:
    
public:
    // Constructor por defecto
    NombreClase();

    // Constructor de copia
    NombreClase(const NombreClase&);

    // Constructor de movimiento
    NombreClase(NombreClase&&) noexcept;

    // Operador de asignacion por copia
    NombreClase& operator=(const NombreClase&);

    // Operador de asignacion por movimiento
    NombreClase& operator=(NombreClase&&) noexcept;

    // Destructor
    ~NombreClase() = default;

};

// --- Implementaciones fuera de la clase ---
// Constructor por defecto
NombreClase::NombreClase() {
    // Constructor por defecto
};

// Destructor
NombreClase::~NombreClase() {
    // Destructor
};

// Constructor de copia
NombreClase::NombreClase(const NombreClase& otro) = default;

// Constructor de movimiento
NombreClase::NombreClase(NombreClase&& otro) noexcept  = default;

// Operador de asignacion por copia
NombreClase& NombreClase::operator=(const NombreClase& otro)  = default;

// Operador de asignacion por movimiento
NombreClase& NombreClase::operator=(NombreClase&& otro) noexcept  = default;

