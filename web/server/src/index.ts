import { Hono } from "hono";
import { PATHS } from "./paths";
import { secureHeaders } from "hono/secure-headers";
import { serveStatic } from "hono/bun";

const app = new Hono();

app.use("*", secureHeaders());
app.get("/*", serveStatic({root: PATHS.PUBLIC}));
