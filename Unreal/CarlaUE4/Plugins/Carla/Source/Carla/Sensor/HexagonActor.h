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
		LifeTime = 0.0f;
		DropSpeed = 0.0f;
		PrimaryActorTick.bCanEverTick = true;
	}
	void CreateHexagonMesh(float Radius);
	~AHexagonActor() 
	{
	}
	void SetBlockageParamter(float LifeTime, float DropSpeed) {
		this->LifeTime = LifeTime;
		this->DropSpeed = DropSpeed;

	}
	virtual void Tick( float DeltaTime) override;
protected:
	virtual void BeginPlay() override;

protected:

private:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Mesh", meta = (AllowPrivateAccess = "true"))
	UProceduralMeshComponent* HexagonMeshComponent;
	float SpawnTime;
	float LifeTime;
	float DropSpeed;



};