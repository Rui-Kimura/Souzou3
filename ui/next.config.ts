/** @type {import('next').NextConfig} */
const nextConfig = {
  async rewrites() {
    return [
      {
        source: '/api/local/:path*',
        destination: 'http://localhost:8100/:path*', 
      },
    ];
  },
};

export default nextConfig;