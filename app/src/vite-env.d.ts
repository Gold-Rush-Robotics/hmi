/// <reference types="vite/client" />

declare module "*.yaml" {
  const data: unknown; // Temporary until this feature is complete
  export default data;
}
