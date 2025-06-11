FROM node:20-alpine AS build


WORKDIR /app

COPY package*.json package-lock.json ./
RUN npm ci

COPY ./ ./

RUN npm run build

FROM nginx:stable-alpine AS production

COPY --from=build /app/nginx /etc/nginx/conf.d
COPY --from=build /app/dist /usr/share/nginx/html

EXPOSE 80

ENTRYPOINT ["nginx", "-g", "daemon off;"] 