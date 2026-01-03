import React from 'react';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';

function Homepage() {
  return (
    <Layout title="Physical AI & Humanoid Robotics" description="A Comprehensive Guide to Embodied Artificial Intelligence">
      <div style={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        minHeight: '80vh',
        padding: '2rem',
        textAlign: 'center',
        background: 'linear-gradient(135deg, #f5f7fa 0%, #e4edf5 100%)',
      }}>
        <div style={{
          maxWidth: '800px',
          margin: '0 auto',
          padding: '2rem',
          backgroundColor: 'white',
          borderRadius: '12px',
          boxShadow: '0 8px 32px rgba(0,0,0,0.1)',
        }}>
          <div style={{
            marginBottom: '1.5rem'
          }}>
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
                transition: 'all 0.3s ease',
                textDecoration: 'none',
                color: 'white',
                borderRadius: '8px'
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
      </div>
    </Layout>
  );
}

export default Homepage;