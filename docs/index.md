---
title: Physical AI & Humanoid Robotics
hide_title: true
---

import useBaseUrl from '@docusaurus/useBaseUrl';

<div style={{
  display: 'flex',
  flexDirection: 'column',
  alignItems: 'center',
  justifyContent: 'center',
  minHeight: '60vh',
  padding: '2rem',
  textAlign: 'center',
  background: 'linear-gradient(135deg, #f5f7fa 0%, #e4edf5 100%)',
  borderRadius: '12px',
  margin: '1rem 0',
  boxShadow: '0 4px 12px rgba(0,0,0,0.05)'
}}>
  <div style={{ marginBottom: '1.5rem' }}>
    <img
      src={useBaseUrl('/img/logo.svg')}
      alt="Physical AI & Humanoid Robotics"
      style={{
        width: '150px',
        height: '150px',
        filter: 'drop-shadow(0 4px 8px rgba(0,0,0,0.1))'
      }}
    />
  </div>

  <h1 style={{
    fontSize: '2.5rem',
    color: '#2e8555',
    margin: '0.5rem 0',
    fontWeight: 'bold'
  }}>
    Physical AI & Humanoid Robotics
  </h1>

  <h2 style={{
    fontSize: '1.5rem',
    color: '#333',
    margin: '0.5rem 0 1.5rem 0',
    fontWeight: 'normal'
  }}>
    A Comprehensive Guide to Embodied Artificial Intelligence
  </h2>

  <p style={{
    fontSize: '1.1rem',
    color: '#555',
    maxWidth: '800px',
    lineHeight: '1.7',
    marginBottom: '2rem'
  }}>
    Welcome to the complete guide on Physical AI and Humanoid Robotics. This book covers the cutting-edge intersection of artificial intelligence and robotics, focusing on creating embodied intelligence through humanoid robots.
  </p>

  <div style={{
    display: 'flex',
    gap: '1rem',
    flexWrap: 'wrap',
    justifyContent: 'center',
    marginTop: '1rem',
    marginBottom: '2rem'
  }}>
    <div style={{
      backgroundColor: 'white',
      padding: '1rem 1.5rem',
      borderRadius: '8px',
      boxShadow: '0 2px 6px rgba(0,0,0,0.08)',
      minWidth: '200px'
    }}>
      <strong style={{ color: '#2e8555' }}>Physical AI</strong><br/>
      <small>Embodied Intelligence</small>
    </div>
    <div style={{
      backgroundColor: 'white',
      padding: '1rem 1.5rem',
      borderRadius: '8px',
      boxShadow: '0 2px 6px rgba(0,0,0,0.08)',
      minWidth: '200px'
    }}>
      <strong style={{ color: '#2e8555' }}>ROS 2</strong><br/>
      <small>Nervous System</small>
    </div>
    <div style={{
      backgroundColor: 'white',
      padding: '1rem 1.5rem',
      borderRadius: '8px',
      boxShadow: '0 2px 6px rgba(0,0,0,0.08)',
      minWidth: '200px'
    }}>
      <strong style={{ color: '#2e8555' }}>Digital Twins</strong><br/>
      <small>Simulation</small>
    </div>
  </div>

  <div style={{ marginTop: '1rem' }}>
    <a
      className="button button--primary button--lg"
      href={useBaseUrl('/docs/chapter-01')}
      style={{
        fontSize: '1.2rem',
        padding: '1rem 2rem',
        backgroundColor: '#2e8555',
        borderColor: '#2e8555',
        fontWeight: 'bold',
        boxShadow: '0 4px 12px rgba(46, 133, 85, 0.3)',
        transition: 'all 0.3s ease'
      }}
      onMouseOver={(e) => {
        e.target.style.transform = 'translateY(-2px)';
        e.target.style.boxShadow = '0 6px 16px rgba(46, 133, 85, 0.4)';
      }}
      onMouseOut={(e) => {
        e.target.style.transform = 'translateY(0)';
        e.target.style.boxShadow = '0 4px 12px rgba(46, 133, 85, 0.3)';
      }}
    >
      📚 Start Reading
    </a>
  </div>
</div>

<div style={{
  display: 'grid',
  gridTemplateColumns: 'repeat(auto-fit, minmax(300px, 1fr))',
  gap: '1.5rem',
  marginTop: '2rem'
}}>
  <div style={{
    backgroundColor: 'white',
    padding: '1.5rem',
    borderRadius: '8px',
    boxShadow: '0 4px 12px rgba(0,0,0,0.08)'
  }}>
    <h3 style={{ color: '#2e8555', marginBottom: '1rem' }}>🎯 What You'll Learn</h3>
    <ul style={{ textAlign: 'left', paddingLeft: '1.5rem' }}>
      <li>Physical AI Fundamentals: Understanding embodied intelligence</li>
      <li>ROS2 Architecture: The nervous system for robots</li>
      <li>Digital Twins: Simulation with Gazebo and Unity</li>
      <li>AI Brain: NVIDIA Isaac for intelligent behavior</li>
      <li>VLA Integration: Vision-Language-Action models</li>
      <li>Capstone Project: Complete autonomous humanoid implementation</li>
    </ul>
  </div>

  <div style={{
    backgroundColor: 'white',
    padding: '1.5rem',
    borderRadius: '8px',
    boxShadow: '0 4px 12px rgba(0,0,0,0.08)'
  }}>
    <h3 style={{ color: '#2e8555', marginBottom: '1rem' }}>👥 Target Audience</h3>
    <p>This book is designed for:</p>
    <ul style={{ textAlign: 'left', paddingLeft: '1.5rem' }}>
      <li>Robotics engineers</li>
      <li>AI researchers</li>
      <li>Software developers interested in robotics</li>
      <li>Students studying AI and robotics</li>
      <li>Anyone interested in the future of embodied AI</li>
    </ul>
  </div>
</div>