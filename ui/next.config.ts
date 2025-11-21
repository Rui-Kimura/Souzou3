import type { NextConfig } from "next";
import { networkInterfaces } from "os";

const getLocalIpAddresses = (): string[] => {
  const nets = networkInterfaces();
  const results: string[] = [];

  for (const name of Object.keys(nets)) {
    const netInterface = nets[name];
    if (netInterface) {
      for (const net of netInterface) {
        if (net.family === 'IPv4' && !net.internal) {
          results.push(net.address);
        }
      }
    }
  }
  return results;
};



const nextConfig: NextConfig = {
  async rewrites() {
    return [
      {
        source: '/api/local/:path*',
        destination: 'http://localhost:8100/:path*', 
      },
    ];
  },
  allowedDevOrigins: [
    "localhost:8100", 
    ...getLocalIpAddresses()
  ],
};

export default nextConfig;