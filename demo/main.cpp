#include <iostream>


#include "deform/gui.hpp"
#include "deform/example.hpp"
#include "deform/ARAP.hpp"

int main(int argc, char* argv[]) {
     MeshDeformerGui gui;

     ExampleMeshDeformer deformer1("1");
     ARAPMeshDeformer deformer2("2");

     AnyParams param1;
     AnyParams param2;

     gui.add_deformer("Stub Deformer 1", deformer1);
     gui.add_deformer("Stub Deformer 2", deformer2);

     gui.add_params("Params 1", param1);
     gui.add_params("Params 2", param2);

     gui.launch("../../resource/armadillo_500.off");

     return 0;
}
