export const ROUTES = {
  DASHBOARD_PAGE: "/",
  CONTROL_PAGE: "/control",
  LOG_PAGE: "/log",
  DETAILS_PAGE: "/details",
};

export type PageMenuItem = {
  text: string;
  icon: JSX.Element;
  url: string;
};
