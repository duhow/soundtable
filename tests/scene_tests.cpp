#include <cassert>
#include <iostream>

#include "core/Scene.h"

// Very small tests for the Scene/Module/Connection model.
// They run as a normal binary and are integrated with CTest.

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

    // Modules can be added, but duplicates should not be inserted.
    assert(scene.AddModule(osc));
    assert(!scene.AddModule(osc));

    assert(scene.AddModule(filter));

    // Valid connection between existing modules.
    Connection connection{.from_module_id = "osc1",
                          .from_port_name = "out",
                          .to_module_id = "filter1",
                          .to_port_name = "in"};
    assert(scene.AddConnection(connection));

    // Duplicated connection should not be added.
    assert(!scene.AddConnection(connection));

    // Remove connection.
    assert(scene.RemoveConnection("osc1", "out", "filter1", "in"));
    assert(!scene.RemoveConnection("osc1", "out", "filter1", "in"));

    // Manage tangible objects.
    scene.UpsertObject(ObjectInstance(1, "osc1", 0.5F, 0.5F, 0.0F));
    assert(scene.objects().size() == 1U);

    scene.UpsertObject(ObjectInstance(1, "osc1", 0.7F, 0.5F, 0.0F));
    // Upsert should replace the existing object instead of inserting a new one.
    assert(scene.objects().size() == 1U);

    scene.RemoveObject(1);
    assert(scene.objects().empty());

    std::cout << "rectai-core-tests: OK" << std::endl;
    return 0;
}
