"use client";

import { useEffect, useState } from "react";

export default function Page() {
const [mapdata, setMapdata] = useState<[]>([]);
const [x,setX] = useState<number>(0);
const [y,setY] = useState<number>(0);

  const fetch_mapdata = async () => {
    const res = await fetch("/api/local/mapdata");
    const data = await res.json();
    setMapdata(data.mapdata);
  };
  const fetch_position = async () => {
    const res = await fetch("/api/local/position");
    const data = await res.json();
    setX(data.x);
    setY(data.y);
  };

  useEffect(()=>{
    fetch_mapdata();
    fetch_position();
    console.log(mapdata)
  },[])

  return (
    <>
      <h2>Map</h2>
      <>{mapdata.map((line : string,index) => {
        let buf :string = line;
        if(index == y)
        {
          const index = x;
          const char = "▲";
          buf = line.substring(0,index) + char + line.substring(index+1);
        }
        buf = buf.replaceAll("0","　");
        buf = buf.replaceAll("1","■");

        return <p key={"ml"+index}>{buf}</p>
      })}</>
    </>
  );
}
