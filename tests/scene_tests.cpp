#include <cassert>
#include <iostream>

#include "core/Scene.h"

// Tests muy simples del modelo Scene/Module/Connection.
// Se ejecutan como binario normal y se integran con CTest.

using rectai::Connection;
using rectai::Module;
using rectai::ModuleKind;
using rectai::ObjectInstance;
using rectai::Scene;

int main()
{
    Scene scene;

    Module osc{"osc1", ModuleKind::kOscillator};
    osc.AddOutputPort("out", true);

    Module filter{"filter1", ModuleKind::kFilter};
    filter.AddInputPort("in", true);

    // Se pueden anadir mdulos.
    assert(scene.AddModule(osc));
    assert(!scene.AddModule(osc));  // duplicado no debera insertarse.

    assert(scene.AddModule(filter));

    // Conexin vlida entre mdulos existentes.
    Connection connection{.from_module_id = "osc1",
                          .from_port_name = "out",
                          .to_module_id = "filter1",
                          .to_port_name = "in"};
    assert(scene.AddConnection(connection));

    // No debera anadir una conexin duplicada.
    assert(!scene.AddConnection(connection));

    // Eliminar conexin.
    assert(scene.RemoveConnection("osc1", "out", "filter1", "in"));
    assert(!scene.RemoveConnection("osc1", "out", "filter1", "in"));

    // Gestionar objetos tangibles.
    scene.UpsertObject(ObjectInstance(1, "osc1", 0.5F, 0.5F, 0.0F));
    assert(scene.objects().size() == 1U);

    scene.UpsertObject(ObjectInstance(1, "osc1", 0.7F, 0.5F, 0.0F));
    assert(scene.objects().size() == 1U);  // upsert reemplaza.

    scene.RemoveObject(1);
    assert(scene.objects().empty());

    std::cout << "rectai-core-tests: OK" << std::endl;
    return 0;
}
