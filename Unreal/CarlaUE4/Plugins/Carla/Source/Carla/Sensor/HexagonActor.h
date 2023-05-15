#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"

#include "HexagonActor.generated.h"

UCLASS()
class AHexagonActor : public AActor
{
	GENERATED_BODY()

public:
	AHexagonActor() 
	{
		HexagonMeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("HexagonMesh"));
	}
	void CreateHexagonMesh(float Radius);
	~AHexagonActor() 
	{
		for (int i = 0; i < 5; i++)
		{
			i = i;
		}
	}
protected:

private:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Mesh", meta = (AllowPrivateAccess = "true"))
	UProceduralMeshComponent* HexagonMeshComponent;
};