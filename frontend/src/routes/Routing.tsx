import { Routes, Route } from "react-router-dom";
import { ROUTES } from "@/types";
import ControlPage from "@/pages/control-page";
import HomePage from "@/pages/home-page";

const Routing = () => {
  return (
    <Routes>
      <Route path={ROUTES.HOME_PAGE} element={<HomePage />} />
      <Route path={ROUTES.CONTROL_PAGE} element={<ControlPage />} />
    </Routes>
  );
};

export default Routing;
