#include "Carla/Sensor/HexagonActor.h"

void AHexagonActor::CreateHexagonMesh(float Radius)
{
    PrimaryActorTick.bCanEverTick = false;

    HexagonMeshComponent->SetMobility(EComponentMobility::Movable);

    const int32 NumVertices = 7;
    const int32 NumTriangles = 6;

    TArray< FVector > Vertices;
    TArray< int32 > Triangles;
    TArray< FVector > Normals;
    TArray< FVector2D > UVs;
    TArray<FLinearColor> Colors;
    TArray< FProcMeshTangent > Tangents;


    // Generate vertices
    float angle = PI / 3.0f;

    for (int i = 0; i < 6; i++)
    {
        float x = Radius * FMath::Cos(angle * i);
        float y = Radius * FMath::Sin(angle * i);
        Vertices.Add(FVector(x, y, 0));
    }
    for (int i = 0; i < Vertices.Num(); i++)
    {
        Normals.Add(FVector(0.f, 0.f, 1.f));
    }
    UVs.Add(FVector2D(0.f, 0.f));
    UVs.Add(FVector2D(1.f, 0.f));
    UVs.Add(FVector2D(1.f, 1.f));
    UVs.Add(FVector2D(0.f, 1.f));
    UVs.Add(FVector2D(0.f, 1.f));
    UVs.Add(FVector2D(0.f, 1.f));
    Colors.Add(FLinearColor::Red);
    Colors.Add(FLinearColor::Red);
    Colors.Add(FLinearColor::Red);
    Colors.Add(FLinearColor::Red);
    Colors.Add(FLinearColor::Red);
    Colors.Add(FLinearColor::Red);
    // Front-facing triangles
    Triangles.Add(0);
    Triangles.Add(1);
    Triangles.Add(2);
    Triangles.Add(0);
    Triangles.Add(2);
    Triangles.Add(3);
    Triangles.Add(0);
    Triangles.Add(3);
    Triangles.Add(4);
    Triangles.Add(0);
    Triangles.Add(4);
    Triangles.Add(5);

    // Create mesh section
    HexagonMeshComponent->RegisterComponent();
    HexagonMeshComponent->CreateMeshSection(0, Vertices, Triangles, Normals, UVs, TArray<FColor>(), Tangents, true);

    RootComponent = HexagonMeshComponent;
    HexagonMeshComponent->SetVisibility(true);

}




