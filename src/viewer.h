/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#ifndef VIEWER_H
#define VIEWER_H

#include <mutex>
#include <string>
#include <vector>
#include <cmath>

#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkSTLReader.h>
#include <vtkTransform.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkProperty.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>
#include <vtkTextActor.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>

#include <Eigen/Eigen>

#include <yarp/sig/PointCloud.h>

namespace viewer {

static std::mutex mtx;

/******************************************************************************/
class UpdateCommand : public vtkCommand {
    bool shutdown{false};

public:
    /**************************************************************************/
    vtkTypeMacro(UpdateCommand, vtkCommand);

    /**************************************************************************/
    static UpdateCommand *New() {
        return new UpdateCommand;
    }

    /**************************************************************************/
    void shutDown() {
        shutdown = true;
    }

    /**************************************************************************/
    void Execute(vtkObject* caller, unsigned long vtkNotUsed(eventId),
                 void* vtkNotUsed(callData)) {
        std::lock_guard<std::mutex> lck(mtx);
        vtkRenderWindowInteractor* iren = static_cast<vtkRenderWindowInteractor*>(caller);
        if (shutdown) {
            iren->GetRenderWindow()->Finalize();
            iren->TerminateApp();
        } else {
            iren->Render();
        }
    }
};

/******************************************************************************/
class Viewer {
    vtkSmartPointer<vtkRenderer>               vtk_renderer{nullptr};
    vtkSmartPointer<vtkRenderWindow>           vtk_renderWindow{nullptr};
    vtkSmartPointer<vtkRenderWindowInteractor> vtk_renderWindowInteractor{nullptr};
    vtkSmartPointer<UpdateCommand>             vtk_updateCallback{nullptr};
    vtkSmartPointer<vtkAxesActor>              vtk_axes{nullptr};
    vtkSmartPointer<vtkInteractorStyleSwitch>  vtk_style{nullptr};
    vtkSmartPointer<vtkCamera>                 vtk_camera{nullptr};
    vtkSmartPointer<vtkPolyDataMapper>         vtk_pc_mapper{nullptr};
    vtkSmartPointer<vtkPoints>                 vtk_pc_points{nullptr};
    vtkSmartPointer<vtkUnsignedCharArray>      vtk_pc_colors{nullptr};
    vtkSmartPointer<vtkPolyData>               vtk_pc_polydata{nullptr};
    vtkSmartPointer<vtkVertexGlyphFilter>      vtk_pc_filter{nullptr};
    vtkSmartPointer<vtkActor>                  vtk_pc_actor{nullptr};
    vtkSmartPointer<vtkSTLReader>              vtk_mdl_reader{nullptr};
    vtkSmartPointer<vtkPolyDataMapper>         vtk_mdl_mapper{nullptr};
    vtkSmartPointer<vtkTransform>              vtk_mdl_transform{nullptr};
    vtkSmartPointer<vtkActor>                  vtk_mdl_actor{nullptr};

public:
    /**************************************************************************/
    Viewer() = delete;

    /**************************************************************************/
    Viewer(const int x, const int y, const int w, const int h) {
        vtk_renderer = vtkSmartPointer<vtkRenderer>::New();
        vtk_renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
        vtk_renderWindow->SetPosition(x, y);
        vtk_renderWindow->SetSize(w, h);
        vtk_renderWindow->SetWindowName("VTK 3D Viewer");
        vtk_renderWindow->AddRenderer(vtk_renderer);
        vtk_renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        vtk_renderWindowInteractor->SetRenderWindow(vtk_renderWindow);
        vtk_renderer->SetBackground(std::vector<double>({.7, .7, .7}).data());

        vtk_axes = vtkSmartPointer<vtkAxesActor>::New();
        vtk_axes->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
        vtk_axes->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
        vtk_axes->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
        vtk_axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
        vtk_axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
        vtk_axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
        vtk_axes->SetTotalLength(std::vector<double>({.1, .1, .1}).data());
        vtk_renderer->AddActor(vtk_axes);

        vtk_style = vtkSmartPointer<vtkInteractorStyleSwitch>::New();
        vtk_style->SetCurrentStyleToTrackballCamera();
        vtk_renderWindowInteractor->SetInteractorStyle(vtk_style);
    }

    /**************************************************************************/
    void start() {
        vtk_renderWindowInteractor->Initialize();
        vtk_renderWindowInteractor->CreateRepeatingTimer(10);
        vtk_updateCallback = vtkSmartPointer<UpdateCommand>::New();
        vtk_renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, vtk_updateCallback);
        vtk_renderWindowInteractor->Start();
    }

    /**************************************************************************/
    void stop() {
        vtk_updateCallback->shutDown();
    }

    /**************************************************************************/
    void setCamera(const std::vector<double>& position, const std::vector<double>& focalpoint,
                   const std::vector<double>& viewup, const double view_angle) {
        std::lock_guard<std::mutex> lck(mtx);
        vtk_camera = vtkSmartPointer<vtkCamera>::New();
        vtk_camera->SetPosition(position.data());
        vtk_camera->SetFocalPoint(focalpoint.data());
        vtk_camera->SetViewUp(viewup.data());
        vtk_camera->SetViewAngle(view_angle);
        vtk_renderer->SetActiveCamera(vtk_camera);
    }

    /**************************************************************************/
    void showPointCloud(std::shared_ptr<yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>> pc) {
        std::lock_guard<std::mutex> lck(mtx);
        if (vtk_pc_actor) {
            vtk_renderer->RemoveActor(vtk_pc_actor);
        }

        vtk_pc_points = vtkSmartPointer<vtkPoints>::New();
        vtk_pc_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        vtk_pc_colors->SetNumberOfComponents(3);

        std::vector<unsigned char> color(3);
        for (size_t i = 0; i < pc->size(); i++) {
            const auto& p = (*pc)(i);
            vtk_pc_points->InsertNextPoint(p.x, p.y, p.z);

            color = {p.r, p.g, p.b};
            vtk_pc_colors->InsertNextTypedTuple(color.data());
        }

        vtk_pc_polydata = vtkSmartPointer<vtkPolyData>::New();
        vtk_pc_polydata->SetPoints(vtk_pc_points);
        vtk_pc_polydata->GetPointData()->SetScalars(vtk_pc_colors);

        vtk_pc_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vtk_pc_filter->SetInputData(vtk_pc_polydata);
        vtk_pc_filter->Update();

        vtk_pc_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_pc_mapper->SetInputConnection(vtk_pc_filter->GetOutputPort());

        vtk_pc_actor = vtkSmartPointer<vtkActor>::New();
        vtk_pc_actor->SetMapper(vtk_pc_mapper);
        vtk_pc_actor->GetProperty()->SetPointSize(1);

        vtk_renderer->AddActor(vtk_pc_actor);
    }

    /**************************************************************************/
    void showModel(const std::string& model, const double scale,
                   const Eigen::Matrix4d& T) {
        std::lock_guard<std::mutex> lck(mtx);
        if (vtk_mdl_actor) {
            vtk_renderer->RemoveActor(vtk_mdl_actor);
        }

        vtk_mdl_reader = vtkSmartPointer<vtkSTLReader>::New();
        vtk_mdl_reader->SetFileName(model.c_str());
        vtk_mdl_reader->Update();

        vtk_mdl_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mdl_mapper->SetInputConnection(vtk_mdl_reader->GetOutputPort());

        vtk_mdl_actor = vtkSmartPointer<vtkActor>::New();
        vtk_mdl_actor->SetMapper(vtk_mdl_mapper);

        Eigen::Vector3d v;
        v(0) = T(2,1) - T(1,2);
        v(1) = T(0,2) - T(2,0);
        v(2) = T(1,0) - T(0,1);
        const auto r = std::sqrt(v(0)*v(0) + v(1)*v(1) + v(2)*v(2));
        const auto angle = (180. / M_PI) * std::atan2(.5 * r, .5 * (T(0,0) + T(1,1) + T(2,2) - 1.));

        vtk_mdl_transform = vtkSmartPointer<vtkTransform>::New();
        vtk_mdl_transform->Translate(T(0,3), T(1,3), T(2,3));
        vtk_mdl_transform->RotateWXYZ(angle, v(0)/r, v(1)/r, v(2)/r);
        vtk_mdl_transform->Scale(scale, scale, scale);
        vtk_mdl_actor->SetUserTransform(vtk_mdl_transform);

        vtk_renderer->AddActor(vtk_mdl_actor);
    }
};

}

#endif
