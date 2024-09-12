#include "MainWindow..h"
#include "imgui.h"
#include "StartCommand.h"

#include <algorithm>

 void MainWindow::Render() 
{
	if (ImGui::Begin("Main Window", 0, ImGuiWindowFlags_NoCollapse)) 
	{
		ImGui::BeginChild("MainButtons", ImVec2(150,0));
		{
			if (ImGui::Button("Shutdown", ImVec2(150, 25)))
			{
				ShutDownRunningProcesses();
				ShutDownWSL WSL;
				ImGui::TextColored(ImVec4(1.f, 1.f, 1.f, 1.f), WSL.GetProcessName());
			}
			if (ImGui::Button("Start Carla", ImVec2(150, 25)))
			{
				ActiveCommands.push_back(new StartCarlaServer);
			}
			if (ImGui::Button("Start ROS-Bridge", ImVec2(150, 25)))
			{
				ActiveCommands.push_back(new StartRosBridge);
			}
			ImGui::BeginEnabled(State & Commands::StartROS);
			{
				if (ImGui::Button("Spawn Objects", ImVec2(150, 25)))
				{
					ActiveCommands.push_back(new ScenarioSpawner);
				}
				ImGui::BeginEnabled(State & Commands::StartCarla);
				{
					if (ImGui::Button("Start Scenario", ImVec2(150, 25)))
					{
						ActiveCommands.push_back(new StartScenarioRunner);
					}
					ImGui::BeginEnabled(State & Commands::StartScenario);
					{
						if (ImGui::Button("Add Scenario", ImVec2(150, 25)))
						{
							ActiveCommands.push_back(new ScenarioAdder);
						}
						ImGui::BeginEnabled(State & Commands::AddScenario && State & Commands::SpawnObjects);
						{
							if (ImGui::Button("Start ManualControl", ImVec2(150, 25)))
							{
								ActiveCommands.push_back(new ManualControl);
							}
							if (ImGui::Button("Start RVIZ", ImVec2(150, 25)))
							{
								ActiveCommands.push_back(new RVIZ);
							}
						}
						ImGui::EndEnabled();
					}
					ImGui::EndEnabled();
				}
				ImGui::EndEnabled();
			}
			ImGui::EndEnabled();
		}
		ImGui::EndChild();
		ImGui::SameLine();
		ImGui::BeginChild("ActiveCommands", ImVec2(150,0));
		{
			for (auto& command : ActiveCommands)
			{
				if (ImGui::Selectable(command->GetProcessName(), SelectedCommand == command))
					SelectedCommand = command;
			}
		}
		ImGui::EndChild();
		if(SelectedCommand != nullptr)
		{
			ImGui::SameLine();
			ImGui::BeginChild("Terminal Data");
			if (ImGui::Button("Kill Process"))
			{
				SelectedCommand->MarkForKill();
			}
			SelectedCommand->Draw();
			ImGui::EndChild();
		}
		ImGui::End();
	}
}

 void MainWindow::Update() 
 {
	 State = 0;
	 for(auto& command : ActiveCommands)
	 {
		 if (command->MarkedForKill())
		 {
			 auto cmd = command;
			 if (command == SelectedCommand)
				 SelectedCommand = nullptr;
			 ActiveCommands.erase(std::remove(ActiveCommands.begin(), ActiveCommands.end(), command), ActiveCommands.end());
			 cmd->ShutDownCommand();
			 delete cmd;
		 }else
			command->UpdateMyState(State);
		 
	 }
 }