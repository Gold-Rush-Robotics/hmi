# Stage 1: Build the app
FROM node:alpine AS builder

WORKDIR /app

COPY app/package*.json ./
COPY app/yarn.lock ./

RUN yarn install

COPY app/ .

RUN yarn build

# Stage 2: Serve the app with Nginx
FROM nginx:alpine

# Copy the built app from the builder stage
COPY --from=builder app/dist /usr/share/nginx/html

# Copy custom Nginx configuration
COPY nginx.conf /etc/nginx/conf.d/default.conf

# Expose port 80
EXPOSE 80

# Start Nginx
CMD ["nginx", "-g", "daemon off;"]