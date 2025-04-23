/// <reference types="vite/client" />

declare module "*.yaml" {
  const data: any; // You can use 'any' for simplicity, but typing it is better
  export default data;
}
