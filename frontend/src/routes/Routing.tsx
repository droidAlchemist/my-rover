import { Routes, Route } from "react-router-dom";
import { ROUTES } from "@/types";
import { ControlPage, DashboardPage, DetailsPage, LogPage } from "@/pages";

export const Routing = () => {
  return (
    <Routes>
      <Route path={ROUTES.DASHBOARD_PAGE} element={<DashboardPage />} />
      <Route path={ROUTES.CONTROL_PAGE} element={<ControlPage />} />
      <Route path={ROUTES.LOG_PAGE} element={<LogPage />} />
      <Route path={ROUTES.DETAILS_PAGE} element={<DetailsPage />} />
    </Routes>
  );
};

export default Routing;
