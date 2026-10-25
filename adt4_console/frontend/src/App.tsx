import {lazy, Suspense} from "react";
import {createHashRouter, Navigate, RouterProvider} from "react-router";
import {Shell} from "./components/Shell";
import {Loading} from "./components/ui";
import {FleetPage} from "./pages/Fleet";
import {LaunchPage} from "./pages/Launch";
import {MapsPage} from "./pages/Maps";
import {NetworkPage} from "./pages/Network";
import {ConfigPage} from "./pages/Config";

const ViewerPage = lazy(() => import("./viewer/ViewerPage"));

const router = createHashRouter([
  {
    path: "/",
    element: <Shell />,
    children: [
      {index: true, element: <Navigate to="/fleet" replace />},
      {path: "fleet", element: <FleetPage />},
      {path: "launch", element: <LaunchPage />},
      {path: "maps", element: <MapsPage />},
      {
        path: "viewer",
        element: (
          <Suspense fallback={<Loading label="Loading 3D viewer" />}>
            <ViewerPage />
          </Suspense>
        ),
      },
      {
        path: "mission",
        element: (
          <Suspense fallback={<Loading label="Loading mission planner" />}>
            <ViewerPage mode="mission" />
          </Suspense>
        ),
      },
      {path: "network", element: <NetworkPage />},
      {path: "config", element: <ConfigPage />},
      {path: "*", element: <Navigate to="/fleet" replace />},
    ],
  },
]);

export function App() {
  return <RouterProvider router={router} />;
}
