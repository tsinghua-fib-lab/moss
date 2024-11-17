FROM node:20-alpine
ENV NODE_ENV=production
WORKDIR /home/node/app

# Install serve
RUN yarn config set registry https://registry.npmmirror.com
RUN yarn global add serve

# Copy build files
RUN mkdir -p /home/node/app/build/docs
COPY ./docs ./build

EXPOSE 80

CMD serve build -p 80
