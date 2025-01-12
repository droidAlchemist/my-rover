export const ROUTES = {
  DASHBOARD_PAGE: "/",
  CONTROL_PAGE: "/control",
  LOG_PAGE: "/log",
  DETAILS_PAGE: "/details",
  SETTINGS_PAGE: "/settings",
};

export type PageMenuItem = {
  text: string;
  icon: JSX.Element;
  url: string;
};
