#!/bin/bash

# Nginx Security Setup for Orange Pi
# This script configures nginx with proper CORS and security headers

echo "Setting up nginx security configuration..."

# Backup existing nginx configuration
if [ -f "/etc/nginx/nginx.conf" ]; then
    cp /etc/nginx/nginx.conf /etc/nginx/nginx.conf.backup.$(date +%s)
    echo "Backed up nginx.conf"
fi

# Create enhanced nginx configuration with security headers
cat << 'EOF' > /etc/nginx/nginx.conf
user www-data;
worker_processes auto;
pid /run/nginx.pid;
include /etc/nginx/modules-enabled/*.conf;

events {
    worker_connections 1024;
    use epoll;
    multi_accept on;
}

http {
    # Basic Settings
    sendfile on;
    tcp_nopush on;
    tcp_nodelay on;
    keepalive_timeout 65;
    types_hash_max_size 2048;
    server_tokens off;

    # Security Headers (applied globally)
    add_header X-Frame-Options "SAMEORIGIN" always;
    add_header X-Content-Type-Options "nosniff" always;
    add_header X-XSS-Protection "1; mode=block" always;
    add_header Referrer-Policy "strict-origin-when-cross-origin" always;
    add_header Content-Security-Policy "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; font-src 'self';" always;

    # MIME Types
    include /etc/nginx/mime.types;
    default_type application/octet-stream;

    # Logging
    log_format main '$remote_addr - $remote_user [$time_local] "$request" '
                    '$status $body_bytes_sent "$http_referer" '
                    '"$http_user_agent" "$http_x_forwarded_for"';

    access_log /var/log/nginx/access.log main;
    error_log /var/log/nginx/error.log warn;

    # Gzip Settings
    gzip on;
    gzip_vary on;
    gzip_proxied any;
    gzip_comp_level 6;
    gzip_types
        text/plain
        text/css
        text/xml
        text/javascript
        application/json
        application/javascript
        application/xml+rss
        application/atom+xml
        image/svg+xml;

    # Rate Limiting
    limit_req_zone $binary_remote_addr zone=api:10m rate=10r/s;
    limit_req_zone $binary_remote_addr zone=cgi:10m rate=5r/s;

    # Include site configurations
    include /etc/nginx/conf.d/*.conf;
    include /etc/nginx/sites-enabled/*;
}
EOF

# Create enhanced site configuration with CORS
cat << 'EOF' > /etc/nginx/sites-available/ros-secure
server {
    listen 80;
    server_name _;
    
    # Rate limiting
    limit_req zone=api burst=20 nodelay;
    
    # Main page - redirect to coptra
    location = / {
        return 301 /coptra/;
    }
    
    # Redirect /coptra to /coptra/ (with trailing slash)
    location = /coptra {
        return 301 /coptra/;
    }

    # Redirect /coptra_blocks to /coptra_blocks/ (with trailing slash)  
    location = /coptra_blocks {
        return 301 /coptra_blocks/;
    }
    
    # Static files from coptra/www
    location /coptra/ {
        alias /var/www/ros/coptra/;
        index index.html;
        try_files $uri $uri/ =404;
        
        # CORS for static files
        add_header 'Access-Control-Allow-Origin' '*' always;
        add_header 'Access-Control-Allow-Methods' 'GET, POST, OPTIONS' always;
        add_header 'Access-Control-Allow-Headers' 'Content-Type, Authorization, X-Requested-With' always;
    }
    
    # Static files from coptra_blocks/www
    location /coptra_blocks/ {
        alias /var/www/ros/coptra_blocks/;
        index index.html;
        try_files $uri $uri/ =404;
        
        # CORS for static files
        add_header 'Access-Control-Allow-Origin' '*' always;
        add_header 'Access-Control-Allow-Methods' 'GET, POST, OPTIONS' always;
        add_header 'Access-Control-Allow-Headers' 'Content-Type, Authorization, X-Requested-With' always;
    }
    
    # For static files (CSS, JS, images) with caching
    location ~* \.(css|js|png|jpg|jpeg|gif|ico|svg)$ {
        root /var/www/ros;
        expires 1y;
        add_header Cache-Control "public, immutable";
        add_header 'Access-Control-Allow-Origin' '*' always;
    }

    # Add favicon
    location = /favicon.ico {
        alias /var/www/ros/coptra/coptra_icon_128.png;
        expires 1y;
    }
    
    # CGI scripts for network management with enhanced security
    location /cgi-bin/ {
        alias /usr/lib/cgi-bin/;
        gzip off;
        
        # Rate limiting for CGI scripts
        limit_req zone=cgi burst=10 nodelay;
        
        # CORS headers for CGI scripts
        add_header 'Access-Control-Allow-Origin' '*' always;
        add_header 'Access-Control-Allow-Methods' 'GET, POST, OPTIONS' always;
        add_header 'Access-Control-Allow-Headers' 'Content-Type, Authorization, X-Requested-With' always;
        add_header 'Access-Control-Max-Age' '3600' always;
        
        # Handle preflight OPTIONS requests
        if ($request_method = 'OPTIONS') {
            add_header 'Access-Control-Allow-Origin' '*' always;
            add_header 'Access-Control-Allow-Methods' 'GET, POST, OPTIONS' always;
            add_header 'Access-Control-Allow-Headers' 'Content-Type, Authorization, X-Requested-With' always;
            add_header 'Access-Control-Max-Age' '3600' always;
            add_header 'Content-Type' 'text/plain; charset=utf-8' always;
            add_header 'Content-Length' '0' always;
            return 204;
        }
        
        # Security headers specific to CGI
        add_header 'X-Content-Type-Options' 'nosniff' always;
        add_header 'X-Frame-Options' 'SAMEORIGIN' always;
        add_header 'X-XSS-Protection' '1; mode=block' always;
        
        fastcgi_pass unix:/var/run/fcgiwrap.socket;
        include /etc/nginx/fastcgi_params;
        fastcgi_param SCRIPT_FILENAME /usr/lib/cgi-bin$fastcgi_script_name;
        
        # FastCGI security parameters
        fastcgi_param HTTP_PROXY "";
        fastcgi_read_timeout 30;
        fastcgi_send_timeout 30;
    }
    
    # Deny access to sensitive files
    location ~ /\. {
        deny all;
        access_log off;
        log_not_found off;
    }
    
    location ~ \.(conf|sh|log)$ {
        deny all;
        access_log off;
        log_not_found off;
    }
    
    # Health check endpoint
    location /health {
        access_log off;
        return 200 "healthy\n";
        add_header Content-Type text/plain;
    }
}
EOF

# Enable the secure configuration and disable older sites to avoid conflicts
ln -sf /etc/nginx/sites-available/ros-secure /etc/nginx/sites-enabled/ros-secure

# Disable default site if it exists
if [ -L "/etc/nginx/sites-enabled/default" ]; then
    rm /etc/nginx/sites-enabled/default
fi

# Disable legacy 'ros' site if present to prevent duplicate location conflicts
if [ -L "/etc/nginx/sites-enabled/ros" ]; then
    rm /etc/nginx/sites-enabled/ros
fi

# Test nginx configuration
if nginx -t; then
    echo "Nginx configuration test passed"
    systemctl reload nginx
    echo "Nginx reloaded with new security configuration"
else
    echo "Nginx configuration test failed, reverting to backup"
    if [ -f "/etc/nginx/nginx.conf.backup" ]; then
        cp /etc/nginx/nginx.conf.backup /etc/nginx/nginx.conf
        systemctl reload nginx
    fi
    exit 1
fi

echo "Nginx security setup completed successfully"
